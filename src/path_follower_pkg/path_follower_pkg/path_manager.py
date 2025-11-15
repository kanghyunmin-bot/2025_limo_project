#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

from .spline_utils import generate_smooth_path, compute_path_curvature
from .ackermann_path_planner import AckermannPathPlanner
from .local_bezier_planner import LocalBezierPlanner

class PathManager:
    def __init__(self, node):
        self.node = node
        self.waypoints = []
        self.global_path = None
        self.local_path = None
        self.velocities = []
        
        self.ackermann_planner = AckermannPathPlanner()
        self.local_bezier_planner = LocalBezierPlanner()
        
        self.use_ackermann_path = False
        self.interpolation_method = 'spline'
        
        self.v_max = 0.5
        self.v_min = 0.15
        self.max_curvature = 2.0
        
        self.sub_use_ackermann = node.create_subscription(
            Bool, '/path_follower/use_ackermann_path', 
            self.on_use_ackermann_path, 10)
        
        self.sub_interpolation_method = node.create_subscription(
            String, '/path_follower/interpolation_method',
            self.on_interpolation_method_change, 10)
        
        self.node.get_logger().info("📍 PathManager: Global Anchor + Local 1m @ 60Hz")

    def on_use_ackermann_path(self, msg: Bool):
        self.use_ackermann_path = msg.data
        if self.use_ackermann_path:
            self.interpolation_method = 'bezier'
        if len(self.waypoints) >= 2:
            self._update_path()
    
    def on_interpolation_method_change(self, msg: String):
        self.interpolation_method = msg.data
        
        if self.interpolation_method != 'local_bezier' and len(self.waypoints) >= 2:
            self._update_path()
        elif self.interpolation_method == 'local_bezier':
            self._generate_global_anchor_path()

    def add_waypoint(self, msg: PointStamped):
        self.waypoints.append((msg.point.x, msg.point.y))
        self.node.get_logger().info(
            f"📍 Waypoint: ({msg.point.x:.2f}, {msg.point.y:.2f}) [Total: {len(self.waypoints)}]"
        )
        if len(self.waypoints) >= 2:
            if self.interpolation_method == 'local_bezier':
                self._generate_global_anchor_path()
            else:
                self._update_path()

    def set_path_from_external(self, path_msg: Path):
        if len(path_msg.poses) < 2:
            return
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        
        if self.interpolation_method == 'local_bezier':
            self._generate_global_anchor_path()
        else:
            self._update_path()
    
    def _generate_global_anchor_path(self):
        """✅ 전역 앵커 경로 생성"""
        if len(self.waypoints) < 2:
            return
        
        try:
            waypoints_np = np.array(self.waypoints)
            smooth_points = generate_smooth_path(waypoints_np, ds=0.05)  # 더 조밀하게
            
            # LocalBezierPlanner에 설정
            self.local_bezier_planner.set_global_path(smooth_points)
            
            # 전역 경로 Path 메시지
            self.global_path = Path()
            self.global_path.header.frame_id = 'odom'
            self.global_path.header.stamp = self.node.get_clock().now().to_msg()
            
            for point in smooth_points:
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.global_path.poses.append(pose)
            
            self.node.get_logger().info(
                f"✅ Global Anchor: {len(self.waypoints)} → {len(smooth_points)} points"
            )
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Global path failed: {e}")
    
    def update_local_bezier_path(self, robot_pos, robot_yaw):
        """✅ 로컬 1m만 실시간 최적화"""
        if self.interpolation_method != 'local_bezier':
            return
        
        if self.global_path is None or len(self.global_path.poses) < 2:
            return
        
        try:
            # ✅ 로봇 현재 위치에서 앞 1m 구간만 베지어 최적화
            curve = self.local_bezier_planner.plan_local_path(
                robot_pos[:2],
                robot_yaw,
                obstacles=[]
            )
            
            # ✅ 로컬 Path = 최적화된 1m 경로만
            self.local_path = Path()
            self.local_path.header.frame_id = 'odom'
            self.local_path.header.stamp = self.node.get_clock().now().to_msg()
            
            for point in curve:
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.local_path.poses.append(pose)
            
            # 속도 계산
            curve_array = np.array(curve)
            curvatures = compute_path_curvature(curve_array)
            self.velocities = self._generate_velocity_profile(curvatures)
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Local Bézier: {e}")
            import traceback
            traceback.print_exc()
    
    def _update_path(self):
        """일반 보간"""
        if len(self.waypoints) < 2:
            return
        try:
            if self.interpolation_method == 'none':
                smooth_points = self._no_interpolation()
            elif self.interpolation_method == 'linear':
                smooth_points = self._linear_interpolation()
            elif self.interpolation_method == 'subsample':
                smooth_points = self._subsample_interpolation()
            elif self.interpolation_method == 'bezier':
                smooth_points = self._bezier_interpolation()
            else:
                smooth_points = self._spline_interpolation()
            
            smooth_points = np.array(smooth_points)
            curvatures = compute_path_curvature(smooth_points)
            self.velocities = self._generate_velocity_profile(curvatures)
            
            self.local_path = Path()
            self.local_path.header.frame_id = 'odom'
            self.local_path.header.stamp = self.node.get_clock().now().to_msg()
            
            for point in smooth_points:
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.local_path.poses.append(pose)
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Path generation: {e}")
    
    def _no_interpolation(self):
        return self.waypoints
    
    def _linear_interpolation(self, ds=0.1):
        path = []
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i+1]
            dist = np.hypot(p2[0]-p1[0], p2[1]-p1[1])
            num_points = max(int(dist / ds), 2)
            for j in range(num_points):
                t = j / (num_points - 1) if num_points > 1 else 0
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                path.append([x, y])
        return np.array(path)
    
    def _subsample_interpolation(self, target_spacing=0.1):
        path = [self.waypoints[0]]
        accumulated_dist = 0.0
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i+1]
            segment_dist = np.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if segment_dist < 1e-6:
                continue
            direction = [(p2[0]-p1[0])/segment_dist, (p2[1]-p1[1])/segment_dist]
            while accumulated_dist + target_spacing < segment_dist:
                accumulated_dist += target_spacing
                new_point = [
                    p1[0] + direction[0] * accumulated_dist,
                    p1[1] + direction[1] * accumulated_dist
                ]
                path.append(new_point)
            accumulated_dist -= segment_dist
        path.append(self.waypoints[-1])
        return np.array(path)
    
    def _spline_interpolation(self):
        waypoints_np = np.array(self.waypoints)
        return generate_smooth_path(waypoints_np, ds=0.1)
    
    def _bezier_interpolation(self):
        return np.array(self.ackermann_planner.plan_path(self.waypoints))

    def _generate_velocity_profile(self, curvatures):
        velocities = []
        for curv in curvatures:
            curv = abs(curv)
            curv = min(curv, self.max_curvature)
            if self.max_curvature > 0:
                velocity = self.v_max - (self.v_max - self.v_min) * (curv / self.max_curvature)
            else:
                velocity = self.v_max
            velocity = max(self.v_min, min(self.v_max, velocity))
            velocities.append(velocity)
        return velocities

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
        self.global_path = None
        self.local_path = None
        self.velocities.clear()
        self.local_bezier_planner.prev_p1 = None
        self.local_bezier_planner.prev_p2 = None
        self.node.get_logger().info("🔄 Reset")
