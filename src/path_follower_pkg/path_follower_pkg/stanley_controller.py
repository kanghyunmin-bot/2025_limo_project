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
        
        # Nearest point
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
        
        # Heading error
        heading_error = normalize_angle(tyaw - robot_yaw)
        
        # Cross-track error
        dx = tx - robot_x
        dy = ty - robot_y
        cross_track_error = -math.sin(tyaw) * dx + math.cos(tyaw) * dy
        
        # Drive mode 확인
        drive_mode = getattr(node, 'drive_mode', 'differential')
        
        if drive_mode == 'differential':
            # ✅ Differential Drive: 제자리 회전 우선
            
            # 1. Heading error가 크면 제자리 회전
            heading_threshold = math.radians(15)  # 15도 이상이면 제자리 회전
            
            if abs(heading_error) > heading_threshold:
                # 제자리 회전 (linear = 0, angular만)
                linear_v = 0.0
                angular_z = 1.5 * heading_error  # 강한 회전
                angular_z = np.clip(angular_z, -2.0, 2.0)
                steering = 0.0
            else:
                # 방향이 맞으면 전진하면서 미세 조정
                linear_v = velocity_ref
                
                # Cross-track error 기반 조향
                v_safe = max(abs(velocity_ref), 0.15)
                steering = heading_error + math.atan2(self.k_e * cross_track_error, v_safe)
                steering = np.clip(steering, -math.radians(30), math.radians(30))
                
                # Angular velocity
                curvature = math.tan(steering) / self.wheelbase
                angular_z = velocity_ref * curvature + 0.3 * heading_error
                angular_z = np.clip(angular_z, -2.0, 2.0)
        
        else:
            # ✅ Ackermann: 표준 Stanley
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
