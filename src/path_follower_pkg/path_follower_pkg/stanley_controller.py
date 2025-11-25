#!/usr/bin/env python3
import math
import numpy as np


def quaternion_to_yaw(q):
    return 2.0 * math.atan2(q.z, q.w)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class StanleyController:
    def __init__(self, k_e=3.5, wheelbase=0.4):
        self.k_e = k_e
        self.wheelbase = wheelbase
        self.last_nearest_idx = 0
        self.initialized = False
    
    def compute_control(self, robot_pose, path_points, velocity_ref, node):
        if len(path_points) < 2:
            return 0.0, 0.0, 0.0
        
        robot_x, robot_y, robot_yaw = robot_pose
        
        if not self.initialized:
            nearest_idx = self._find_nearest_global(robot_x, robot_y, path_points)
            self.initialized = True
        else:
            nearest_idx = self._find_nearest_local(robot_x, robot_y, path_points)
        
        self.last_nearest_idx = nearest_idx
        
        target = path_points[nearest_idx]
        tx = target.pose.position.x
        ty = target.pose.position.y
        tyaw = quaternion_to_yaw(target.pose.orientation)
        
        heading_error = normalize_angle(tyaw - robot_yaw)
        
        dx = tx - robot_x
        dy = ty - robot_y
        cross_track_error = -math.sin(tyaw) * dx + math.cos(tyaw) * dy
        
        drive_mode = getattr(node, 'drive_mode', 'differential')
        
        if drive_mode == 'differential':
            # ✅ 완화된 제자리 회전 조건
            heading_threshold = math.radians(25)  # 15° → 25° (더 관대하게)
            
            # ✅ 속도에 따른 동적 threshold
            speed_factor = max(abs(velocity_ref), 0.1)
            adjusted_threshold = heading_threshold * (0.5 / speed_factor)  # 속도 빠르면 threshold 높음
            
            if abs(heading_error) > adjusted_threshold and abs(cross_track_error) > 0.03:
                # 제자리 회전 (방향 오차 크고 + 경로 많이 벗어남)
                linear_v = 0.0
                angular_z = 1.2 * heading_error  # 1.5 → 1.2 (약하게)
                angular_z = np.clip(angular_z, -2.0, 2.0)
                steering = 0.0
            else:
                # 전진하면서 조정
                linear_v = velocity_ref
                
                v_safe = max(abs(velocity_ref), 0.15)
                steering = heading_error + math.atan2(self.k_e * cross_track_error, v_safe)
                steering = np.clip(steering, -math.radians(30), math.radians(30))
                
                curvature = math.tan(steering) / self.wheelbase
                angular_z = velocity_ref * curvature + 0.3 * heading_error
                angular_z = np.clip(angular_z, -2.0, 2.0)
        
        else:
            # Ackermann
            linear_v = velocity_ref
            
            v_safe = max(abs(velocity_ref), 0.15)
            steering = heading_error + math.atan2(self.k_e * cross_track_error, v_safe)
            steering = np.clip(steering, -math.radians(30), math.radians(30))
            
            angular_z = (2.0 * velocity_ref * math.sin(steering)) / self.wheelbase
            angular_z = np.clip(angular_z, -2.0, 2.0)
        
        return linear_v, angular_z, steering
    
    def _find_nearest_global(self, rx, ry, path_points):
        min_dist = float('inf')
        nearest = 0
        
        for i in range(len(path_points)):
            px = path_points[i].pose.position.x
            py = path_points[i].pose.position.y
            dist = math.hypot(px - rx, py - ry)
            
            if dist < min_dist:
                min_dist = dist
                nearest = i
        
        return nearest
    
    def _find_nearest_local(self, rx, ry, path_points):
        min_dist = float('inf')
        nearest = self.last_nearest_idx
        
        start = max(0, self.last_nearest_idx - 5)
        end = min(len(path_points), self.last_nearest_idx + 40)
        
        for i in range(start, end):
            px = path_points[i].pose.position.x
            py = path_points[i].pose.position.y
            dist = math.hypot(px - rx, py - ry)
            
            if dist < min_dist:
                min_dist = dist
                nearest = i
        
        return nearest
    
    def reset(self):
        self.last_nearest_idx = 0
        self.initialized = False
