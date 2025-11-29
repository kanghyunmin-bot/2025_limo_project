#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

from .spline_utils import compute_path_curvature, generate_smooth_path
from .ackermann_path_planner import AckermannPathPlanner
from .rrt_planner import RRTPlanner

# ✅ 조건부 import
try:
    from .bezier_utils import generate_bezier_from_waypoints, split_global_to_local_bezier
    BEZIER_AVAILABLE = True
except ImportError:
    BEZIER_AVAILABLE = False


def quaternion_from_yaw(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


class PathManager:
    def __init__(self, node):
        self.node = node
        self.waypoints = []
        self.robot_start_pos = None
        self.global_path = None
        self.local_path = None
        self.velocities = []
        self.global_curvatures = []
        self.constraint_points = []
        self.global_constraints = []
        self.global_constraint_window = 0.8
        self.global_obstacles = []
        self.global_obstacle_window = 1.0
        self.global_obstacle_cap = 32

        self.ackermann_planner = AckermannPathPlanner()
        self.rrt_planner = RRTPlanner()
        self.interpolation_method = 'spline'
        self.drive_mode = 'differential'
        self.use_ackermann_path = False
        
        self.v_max = 1.0
        self.v_min = 0.2
        self.max_lateral_accel = 2.0
        
        self.sub_interpolation_method = node.create_subscription(
            String, '/path_follower/interpolation_method',
            self.on_interpolation_method_change, 10)
        
        self.sub_drive_mode = node.create_subscription(
            String, '/path_follower/drive_mode',
            self.on_drive_mode_change, 10)
        
        self.sub_use_ackermann = node.create_subscription(
            Bool, '/path_follower/use_ackermann_path',
            self.on_use_ackermann_path, 10)
        
        self.node.get_logger().info("📍 PathManager initialized")
    
    def on_interpolation_method_change(self, msg: String):
        self.interpolation_method = msg.data
        self.node.get_logger().info(f"🛣️ Interpolation: {msg.data}")
        if len(self.waypoints) >= 2:
            self._update_path()
    
    def on_drive_mode_change(self, msg: String):
        if msg.data in ['differential', 'ackermann']:
            old_mode = self.drive_mode
            self.drive_mode = msg.data
            self.node.get_logger().info(f"🔄 PathManager: {old_mode} → {self.drive_mode}")
            
            if len(self.waypoints) >= 2:
                self._update_path()
    
    def on_use_ackermann_path(self, msg: Bool):
        self.use_ackermann_path = msg.data
        if len(self.waypoints) >= 2:
            self._update_path()

    def add_waypoint(self, msg: PointStamped):
        self.waypoints.append((msg.point.x, msg.point.y))
        self.node.get_logger().info(f"📍 Waypoint {len(self.waypoints)}")
        if len(self.waypoints) >= 2:
            self._update_path()

    def set_path_from_external(self, path_msg: Path):
        if len(path_msg.poses) < 2:
            return
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        self._update_path()
    
    def _compute_path_orientations(self, points):
        orientations = []
        lookahead = 3
        
        for i in range(len(points)):
            j = min(i + lookahead, len(points) - 1)
            
            if j == i:
                yaw = orientations[-1] if orientations else 0.0
            else:
                dx = points[j][0] - points[i][0]
                dy = points[j][1] - points[i][1]
                yaw = math.atan2(dy, dx)
            
            orientations.append(yaw)
        
        return orientations
    
    def _create_pose_with_orientation(self, point, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = 0.0
        
        q = quaternion_from_yaw(yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose
    
    def _update_path(self):
        if len(self.waypoints) < 2:
            return

        try:
            ds = 0.08

            waypoints_to_use = self.waypoints.copy()
            if self.robot_start_pos is not None:
                waypoints_to_use[0] = (self.robot_start_pos[0], self.robot_start_pos[1])

            if self.global_obstacles and self.interpolation_method in ['bezier', 'local_bezier']:
                waypoints_to_use = self._rrt_bridge_waypoints(waypoints_to_use)

            if self.interpolation_method == 'none':
                smooth_points = np.array(waypoints_to_use)
            elif self.interpolation_method == 'linear':
                smooth_points = self._linear_interpolation(waypoints_to_use, ds=ds)
            elif self.interpolation_method == 'subsample':
                smooth_points = self._subsample_interpolation(waypoints_to_use, ds=ds)
            elif self.interpolation_method == 'bezier':
                smooth_points = np.array(self.ackermann_planner.plan_path(waypoints_to_use))
            elif self.interpolation_method == 'local_bezier':
                waypoints_np = np.array(waypoints_to_use)
                if BEZIER_AVAILABLE:
                    smooth_points = generate_bezier_from_waypoints(
                        waypoints_np,
                        ds=ds,
                        constraints=self.global_constraints,
                        constraint_window=self.global_constraint_window,
                        obstacles=self.global_obstacles,
                        obstacle_window=self.global_obstacle_window,
                        obstacle_cap=self.global_obstacle_cap,
                    )
                    if smooth_points is None:
                        self.node.get_logger().warn(
                            "⚠️ local_bezier: 제어점이 부족해 spline으로 대체합니다."
                        )
                        smooth_points = generate_smooth_path(waypoints_np, ds=ds)
                else:
                    smooth_points = generate_smooth_path(waypoints_np, ds=ds)
            else:
                waypoints_np = np.array(waypoints_to_use)
                smooth_points = generate_smooth_path(waypoints_np, ds=ds)
            
            smooth_points = np.array(smooth_points)
            orientations = self._compute_path_orientations(smooth_points)
            
            self.global_path = Path()
            self.global_path.header.frame_id = 'odom'
            self.global_path.header.stamp = self.node.get_clock().now().to_msg()
            
            for point, yaw in zip(smooth_points, orientations):
                pose = self._create_pose_with_orientation(point, yaw)
                self.global_path.poses.append(pose)
            
            # ✅ Drive mode에 따라 속도 설정
            if self.drive_mode == 'differential':
                self.velocities = [self.v_max] * len(smooth_points)
                self.node.get_logger().info(
                    f"✅ Path: {len(smooth_points)} pts | Differential (uniform v={self.v_max:.2f})"
                )
            else:
                curvatures = compute_path_curvature(smooth_points)
                self.velocities = self._simple_velocity_profile(curvatures)
                self.global_curvatures = curvatures
                self.node.get_logger().info(
                    f"✅ Path: {len(smooth_points)} pts | Ackermann (curvature-based)"
                )
            
            self.local_path = self.global_path

        except Exception as e:
            self.node.get_logger().error(f"❌ {e}")

    def _rrt_bounds(self, p0: np.ndarray, p1: np.ndarray):
        low = np.minimum(p0, p1) - (self.global_obstacle_window + 0.6)
        high = np.maximum(p0, p1) + (self.global_obstacle_window + 0.6)
        return low, high

    def _rrt_bridge_waypoints(self, waypoints):
        """Waypoint 사이를 RRT로 연결해 코스트맵 장애물을 피해가는 얇은 경로를 만든다."""

        if len(waypoints) < 2:
            return waypoints

        planned: list[np.ndarray] = [np.array(waypoints[0], dtype=float)]
        for i in range(len(waypoints) - 1):
            start = planned[-1]
            goal = np.array(waypoints[i + 1], dtype=float)
            # 세그먼트 인근 장애물만 추려 RRT 충돌 검사를 단순화
            seg_mid = 0.5 * (start + goal)
            seg_len = np.linalg.norm(goal - start) + 1e-6
            seg_obs = [
                (c, r)
                for (c, r) in self.global_obstacles
                if np.linalg.norm(c - seg_mid) <= seg_len + self.global_obstacle_window
            ]
            rrt_path = self.rrt_planner.plan(
                start,
                goal,
                obstacles=seg_obs,
                bounds=self._rrt_bounds(start, goal),
            )
            if not rrt_path:
                planned.append(goal)
                continue
            # 첫 점은 이미 planned[-1]에 있으므로 제외
            for pt in rrt_path[1:]:
                planned.append(pt)

        return [(float(p[0]), float(p[1])) for p in planned]

    def update_local_path_with_cp(self, robot_pos):
        """✅ Local Bézier 실시간 업데이트"""
        if self.interpolation_method != 'local_bezier' or not BEZIER_AVAILABLE:
            return
        
        if self.global_path is None or len(self.global_path.poses) < 2:
            return
        
        try:
            lookahead = 0.6
            if self.constraint_points:
                nearest_cp = self.nearest_constraint_distance(robot_pos)
                if nearest_cp is None:
                    nearest_cp = 0.8
                extra = np.clip(1.0 - nearest_cp, 0.0, 0.8)
                lookahead += extra

            local_bezier_points = split_global_to_local_bezier(
                self.global_path,
                robot_pos,
                lookahead_dist=lookahead,
                constraints=self.constraint_points,
            )
            
            if local_bezier_points is None or len(local_bezier_points) < 2:
                return
            
            self.local_path = Path()
            self.local_path.header.frame_id = 'odom'
            self.local_path.header.stamp = self.node.get_clock().now().to_msg()
            
            orientations = self._compute_path_orientations(local_bezier_points)
            
            for point, yaw in zip(local_bezier_points, orientations):
                pose = self._create_pose_with_orientation(point, yaw)
                self.local_path.poses.append(pose)
            
        except Exception as e:
            pass
    
    def _linear_interpolation(self, waypoints, ds=0.08):
        path = []
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i+1]
            dist = np.hypot(p2[0]-p1[0], p2[1]-p1[1])
            num_points = max(int(dist / ds), 2)
            for j in range(num_points):
                t = j / (num_points - 1) if num_points > 1 else 0
                path.append([p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1])])
        return np.array(path)
    
    def _subsample_interpolation(self, waypoints, ds=0.08):
        path = [waypoints[0]]
        accumulated_dist = 0.0
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i+1]
            segment_dist = np.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if segment_dist < 1e-6:
                continue
            direction = [(p2[0]-p1[0])/segment_dist, (p2[1]-p1[1])/segment_dist]
            while accumulated_dist + ds < segment_dist:
                accumulated_dist += ds
                path.append([
                    p1[0] + direction[0] * accumulated_dist,
                    p1[1] + direction[1] * accumulated_dist
                ])
            accumulated_dist -= segment_dist
        path.append(waypoints[-1])
        return np.array(path)
    
    def _simple_velocity_profile(self, curvatures):
        velocities = []
        for curv in curvatures:
            if abs(curv) < 1e-4:
                v = self.v_max
            else:
                v = min(self.v_max, math.sqrt(self.max_lateral_accel / abs(curv)) * 0.9)
            velocities.append(np.clip(v, self.v_min, self.v_max))
        return velocities
    
    def create_initial_path(self, robot_pos):
        try:
            self.robot_start_pos = robot_pos
            self.waypoints = [
                (robot_pos[0], robot_pos[1]),
                (robot_pos[0] + 1.0, robot_pos[1])
            ]
            self._update_path()
            return True
        except Exception as e:
            self.node.get_logger().error(f"❌ {e}")
            return False
    
    def set_robot_start(self, robot_pos):
        self.robot_start_pos = robot_pos

    def update_constraint_points(self, constraint_points):
        self.constraint_points = constraint_points

    def update_global_constraints(self, constraint_points, window: float | None = None, replan: bool = False):
        self.global_constraints = constraint_points or []
        if window is not None:
            self.global_constraint_window = float(window)
        if replan and len(self.waypoints) >= 2:
            self._update_path()

    def update_global_obstacles(self, obstacles, replan: bool = False):
        self.global_obstacles = obstacles or []
        if replan and len(self.waypoints) >= 2:
            self._update_path()

    def nearest_constraint_distance(self, robot_pos):
        if not self.constraint_points:
            return None

        robot_xy = np.array(robot_pos)
        return min(np.linalg.norm(cp - robot_xy) for cp in self.constraint_points)

    def get_local_path(self):
        return self.local_path
    
    def get_global_path(self):
        return self.global_path

    def get_velocity_at_index(self, idx):
        if not self.velocities or idx >= len(self.velocities):
            return self.v_max
        return self.velocities[idx]

    def reset(self):
        self.waypoints.clear()
        self.robot_start_pos = None
        self.global_path = None
        self.local_path = None
        self.velocities.clear()
        self.global_curvatures.clear()
        self.constraint_points.clear()
        self.global_constraints.clear()
        self.global_obstacles.clear()
