#!/usr/bin/env python3
import math
import numpy as np
from .path_utils import (
    find_nearest_point_on_path,
    quaternion_to_yaw,
    normalize_angle
)

class PathController:
    def __init__(self, k_ld=0.6, k_theta=1.0, wheelbase=0.4):
        self.k_ld = k_ld        # ✅ Lookahead 계수
        self.k_theta = k_theta  # ✅ Orientation 게인 (높게)
        self.wheelbase = wheelbase
        self.last_nearest_idx = 0
        self.min_velocity = 0.01
    
    def compute_control(self, robot_pose, path_points, velocity_ref, node):
        if len(path_points) < 2:
            return 0.0, 0.0, 0.0
        
        robot_x, robot_y, robot_yaw = robot_pose
        robot_pos = [robot_x, robot_y]
        
        # ✅ 1. Nearest point
        nearest_idx, _ = find_nearest_point_on_path(
            robot_pos, robot_yaw, path_points, self.last_nearest_idx
        )
        self.last_nearest_idx = nearest_idx
        
        # ✅ 2. Lookahead distance (적당히)
        lookahead_dist = self.k_ld * velocity_ref + 0.3
        lookahead_dist = np.clip(lookahead_dist, 0.3, 1.5)
        
        # ✅ 3. Lookahead point
        accumulated = 0.0
        target_idx = nearest_idx
        
        for i in range(nearest_idx, len(path_points) - 1):
            p1 = path_points[i].pose.position
            p2 = path_points[i+1].pose.position
            seg_len = math.hypot(p2.x - p1.x, p2.y - p1.y)
            accumulated += seg_len
            if accumulated >= lookahead_dist:
                target_idx = i + 1
                break
        
        target_idx = min(target_idx, len(path_points) - 1)
        target_pose = path_points[target_idx]
        
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_yaw = quaternion_to_yaw(target_pose.pose.orientation)
        
        # ✅ 4. Pure Pursuit angle
        alpha = math.atan2(target_y - robot_y, target_x - robot_x) - robot_yaw
        alpha = normalize_angle(alpha)
        
        # ✅ 5. Orientation error (강하게)
        orientation_error = normalize_angle(target_yaw - robot_yaw)
        
        # ✅ 6. Steering angle
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead_dist)
        steering += self.k_theta * orientation_error  # ✅ 방향 강화
        
        max_steering = math.radians(35)
        steering = np.clip(steering, -max_steering, max_steering)
        
        # ✅ 7. Angular velocity
        v_safe = max(abs(velocity_ref), self.min_velocity)
        angular_z = (2.0 * v_safe * math.sin(steering)) / self.wheelbase
        angular_z = np.clip(angular_z, -2.5, 2.5)
        
        return velocity_ref, angular_z, steering
