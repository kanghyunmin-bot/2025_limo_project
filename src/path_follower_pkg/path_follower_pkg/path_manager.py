#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

from .spline_utils import compute_path_curvature, generate_smooth_path
from .ackermann_path_planner import AckermannPathPlanner
from .apf_planner import APFPlanner
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
        self.local_constraint_window = 0.33
        self.global_obstacles = []
        self.global_obstacle_window = 1.0
        self.global_obstacle_cap = 32
        self.freeze_global_path = True
        self.global_locked = False
        self.rrt_path_generated = False  # RRT 경로 생성 여부 확인 플래그
        self.planned_waypoints: list[tuple[float, float]] = []
        self._path_dirty = True

        self.ackermann_planner = AckermannPathPlanner()
        self.rrt_planner = RRTPlanner()
        self.apf_planner = APFPlanner()
        self.interpolation_method = 'spline'
        self.drive_mode = 'differential'
        self.use_ackermann_path = False
        self.planner_mode = 'astar'
        
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
        self._path_dirty = True
        if len(self.waypoints) >= 2:
            self._update_path()
    
    def on_drive_mode_change(self, msg: String):
        if msg.data in ['differential', 'ackermann']:
            old_mode = self.drive_mode
            self.drive_mode = msg.data
            self.node.get_logger().info(f"🔄 PathManager: {old_mode} → {self.drive_mode}")
            self._path_dirty = True

            if len(self.waypoints) >= 2:
                self._update_path()
    
    def on_use_ackermann_path(self, msg: Bool):
        self.use_ackermann_path = msg.data
        self._path_dirty = True
        if len(self.waypoints) >= 2:
            self._update_path()

    def add_waypoint(self, msg: PointStamped):
        self.waypoints.append((msg.point.x, msg.point.y))
        self.node.get_logger().info(f"📍 Waypoint {len(self.waypoints)}")
        self.rrt_path_generated = False  # 새로운 목적지면 플래그 해제
        self._path_dirty = True
        self.global_locked = False
        if len(self.waypoints) >= 2:
            self._update_path()

    def set_path_from_external(self, path_msg: Path):
        if len(path_msg.poses) < 2:
            return
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        self._path_dirty = True
        self.global_locked = False
        self.rrt_path_generated = False
        self._update_path()

    def _update_local_path_only(self):
        """RRT는 그대로 두고 곡선/회피만 다시 계산"""
        if self.global_path is None:
            return

        waypoints_to_use = [
            (pose.pose.position.x, pose.pose.position.y) for pose in self.global_path.poses
        ]
        self._apply_interpolation(waypoints_to_use)
    
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
        """전체 경로 생성 파이프라인 (통합 버전)"""
        if len(self.waypoints) < 2:
            return

        # 이미 생성된 글로벌 경로가 잠겨 있으면 재계산하지 않는다.
        if self.freeze_global_path and self.global_path is not None and self.global_locked:
            self.local_path = self.global_path
            self._path_dirty = False
            return

        # 1) Planner 단계: 한 번 생성된 경로는 재계산을 피한다.
        if not self.rrt_path_generated:
            success = False

            if self.planner_mode == 'rrt':
                success = self._rrt_bridge_waypoints()
            elif self.planner_mode == 'apf':
                success = self._apf_bridge_waypoints()
            elif self.planner_mode == 'ackermann':
                success = self._ackermann_bridge_waypoints()
            else:
                self._convert_list_to_global_path(self.waypoints)
                success = True

            if success:
                self.rrt_path_generated = True

        # 2) Interpolation 단계: 뼈대 경로 위에서 스무딩
        if self.global_path is not None:
            self._apply_interpolation()

        self._path_dirty = False

    def _apply_interpolation(self, waypoints_override=None):
        ds = 0.16 if self.interpolation_method not in ['local_bezier', 'only_global_bezier'] else 0.08

        if waypoints_override is not None:
            waypoints_to_use = list(waypoints_override)
        elif self.global_path is not None:
            waypoints_to_use = [
                (pose.pose.position.x, pose.pose.position.y) for pose in self.global_path.poses
            ]
        else:
            return

        if self.robot_start_pos is not None and waypoints_to_use:
            waypoints_to_use[0] = (self.robot_start_pos[0], self.robot_start_pos[1])

        self.planned_waypoints = waypoints_to_use.copy()

        if self.interpolation_method == 'none':
            smooth_points = np.array(waypoints_to_use)
        elif self.interpolation_method == 'linear':
            smooth_points = self._linear_interpolation(waypoints_to_use, ds=ds)
        elif self.interpolation_method == 'subsample':
            smooth_points = self._subsample_interpolation(waypoints_to_use, ds=ds)
        elif self.interpolation_method == 'bezier':
            smooth_points = np.array(self.ackermann_planner.plan_path(waypoints_to_use))
        elif self.interpolation_method == 'only_global_bezier':
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
                        "⚠️ only_global_bezier: 제어점이 부족해 spline으로 대체합니다."
                    )
                    smooth_points = generate_smooth_path(waypoints_np, ds=ds)
            else:
                smooth_points = generate_smooth_path(waypoints_np, ds=ds)
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

        self._calculate_curvatures_and_velocities(smooth_points)

        if self.drive_mode == 'differential':
            self.node.get_logger().info(
                f"✅ Path: {len(smooth_points)} pts | Differential (decel-aware profile)"
            )
        else:
            self.node.get_logger().info(
                f"✅ Path: {len(smooth_points)} pts | Ackermann (curvature+distance profile)"
            )

        self.local_path = self.global_path
        if self.freeze_global_path:
            self.global_locked = True
        self._path_dirty = False

    def _rrt_bounds(self, p0: np.ndarray, p1: np.ndarray):
        low = np.minimum(p0, p1) - (self.global_obstacle_window + 0.6)
        high = np.maximum(p0, p1) + (self.global_obstacle_window + 0.6)
        return low, high

    def _rrt_bridge_waypoints(self):
        """RRT: 장애물(반지름 포함)을 고려해 웨이포인트 연결"""
        new_path = []
        new_path.append(self.waypoints[0])

        rrt_obstacles = []
        obstacle_radius = 0.30
        for obs in self.global_obstacles:
            if isinstance(obs, (list, tuple)) and len(obs) == 2 and np.isscalar(obs[1]):
                center = np.array(obs[0], dtype=float)
            else:
                center = np.array(obs, dtype=float)
            rrt_obstacles.append((center, obstacle_radius))

        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]

            segment = self.rrt_planner.plan(start, end, obstacles=rrt_obstacles)

            if segment:
                new_path.extend(segment[1:])
            else:
                self.node.get_logger().warn(f"RRT failed segment {i}")
                new_path.append(end)

        self._convert_list_to_global_path(new_path)
        return True

    def _apf_bridge_waypoints(self):
        """APF: 인공 포텐셜 필드로 장애물 회피 경로 생성"""
        new_path = []
        new_path.append(self.waypoints[0])

        apf_obstacles = []
        obstacle_radius = 0.30
        for obs in self.global_obstacles:
            if isinstance(obs, (list, tuple)) and len(obs) == 2 and np.isscalar(obs[1]):
                center = np.array(obs[0], dtype=float)
            else:
                center = np.array(obs, dtype=float)
            apf_obstacles.append((center, obstacle_radius))

        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]

            segment = self.apf_planner.plan(start, end, obstacles=apf_obstacles)

            if segment:
                new_path.extend(segment[1:])
            else:
                new_path.append(end)

        self._convert_list_to_global_path(new_path)
        return True

    def _ackermann_bridge_waypoints(self):
        """Ackermann: 차량 운동학 기반 연결"""
        new_points = self.ackermann_planner.plan_path(self.waypoints)
        self._convert_list_to_global_path(new_points)
        return True

    def _convert_list_to_global_path(self, points_list):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        for xy in points_list:
            pose = PoseStamped()
            pose.pose.position.x = xy[0]
            pose.pose.position.y = xy[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.global_path = path_msg

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
                constraint_window=self.local_constraint_window,
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

    def _calculate_curvatures_and_velocities(self, points):
        if len(points) < 3:
            self.global_curvatures = [0.0] * len(points)
            self.velocities = [self.v_min] * len(points)
            return

        self.global_curvatures = compute_path_curvature(np.array(points))
        self.velocities = []

        # 누적 거리로 남은 거리를 추적해 도착 직전에 선형 감속
        dists = [0.0] * len(points)
        for i in range(1, len(points)):
            d = math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
            dists[i] = dists[i - 1] + d

        total_dist = dists[-1]
        decel_dist = 1.0  # 도착 1m 전부터 감속 시작

        for i, k in enumerate(self.global_curvatures):
            v_curv = self.v_max / (1.0 + 3.0 * abs(k))

            remaining_dist = total_dist - dists[i]
            if remaining_dist < decel_dist:
                ratio = remaining_dist / decel_dist if decel_dist > 0 else 0.0
                v_dist = self.v_max * ratio
                v_final = min(v_curv, v_dist)
                v_final = max(v_final, 0.15)
            else:
                v_final = v_curv

            v_final = max(self.v_min, min(self.v_max, v_final))
            self.velocities.append(v_final)
    
    def create_initial_path(self, robot_pos):
        try:
            self.robot_start_pos = robot_pos
            self._path_dirty = True
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
        self._path_dirty = True
        self.global_locked = False

    def update_constraint_points(self, constraint_points):
        self.constraint_points = constraint_points

    def update_global_constraints(self, constraint_points, window: float | None = None, replan: bool = False):
        self.global_constraints = constraint_points or []
        if window is not None:
            self.global_constraint_window = float(window)
        if replan and len(self.waypoints) >= 2:
            self._update_local_path_only()

    def update_global_obstacles(self, obstacles, replan: bool = False):
        self.global_obstacles = obstacles or []
        if replan and self.rrt_path_generated:
            self._update_local_path_only()
        elif replan and not self.rrt_path_generated and len(self.waypoints) >= 2:
            self._path_dirty = True
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
        self.global_locked = False
        self.planned_waypoints.clear()
        self.rrt_path_generated = False
        self._path_dirty = True
