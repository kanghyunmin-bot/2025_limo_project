#!/usr/bin/env python3
import math

class StanleyController:
    """Stanley Method 제어기"""
    
    def __init__(self, k_e=1.0, k_h=0.5, wheelbase=0.4):
        """
        Args:
            k_e: Cross-track error 게인
            k_h: Heading error 게인
            wheelbase: 축간 거리
        """
        self.k_e = k_e
        self.k_h = k_h
        self.wheelbase = wheelbase
    
    def compute_control(self, robot_pose, path, target_speed, node):
        """
        Stanley Method 제어 계산
        
        Args:
            robot_pose: [x, y, theta]
            path: [[x1, y1], [x2, y2], ...]
            target_speed: 목표 속도
            node: ROS2 노드
        
        Returns:
            linear_v: 선속도
            angular_z: 각속도
            steering_angle: 조향각
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.0
        
        x, y, theta = robot_pose
        
        # 가장 가까운 경로점 찾기
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(path):
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 경로 방향 계산
        if closest_idx < len(path) - 1:
            px1, py1 = path[closest_idx]
            px2, py2 = path[closest_idx + 1]
            path_heading = math.atan2(py2 - py1, px2 - px1)
        else:
            if closest_idx > 0:
                px1, py1 = path[closest_idx - 1]
                px2, py2 = path[closest_idx]
                path_heading = math.atan2(py2 - py1, px2 - px1)
            else:
                path_heading = 0.0
        
        # Heading error
        heading_error = path_heading - theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Cross-track error
        px, py = path[closest_idx]
        dx = px - x
        dy = py - y
        
        cross_track_error = -math.sin(path_heading) * dx + math.cos(path_heading) * dy
        
        # Stanley control law
        steering_angle = (self.k_h * heading_error + 
                         math.atan(self.k_e * cross_track_error / (target_speed + 0.1)))
        
        # Steering angle 제한
        max_steering = math.radians(30)
        steering_angle = max(-max_steering, min(max_steering, steering_angle))
        
        # Angular velocity 계산
        angular_z = target_speed * math.tan(steering_angle) / self.wheelbase
        
        return target_speed, angular_z, steering_angle
