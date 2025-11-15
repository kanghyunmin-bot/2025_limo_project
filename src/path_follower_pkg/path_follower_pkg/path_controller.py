#!/usr/bin/env python3
import math
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time


class PathController:
    def __init__(self, k_ld=0.5, wheelbase=0.4):
        self.k_ld = k_ld
        self.wheelbase = wheelbase
    
    def compute_control(self, robot_pose, path, target_speed, node):
        """
        Pure Pursuit 제어
        robot_pose: [x, y, theta]
        path: [[x1, y1], [x2, y2], ...]
        target_speed: 목표 속도
        node: ROS2 노드 (시간 정보용)
        """
        if len(path) < 2:
            return 0.0, 0.0, None
        
        x, y, theta = robot_pose
        
        # Lookahead distance
        lookahead_distance = self.k_ld * target_speed + 0.3
        lookahead_distance = max(lookahead_distance, 0.5)
        
        # 가장 가까운 경로점 찾기
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(path):
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Lookahead point 찾기
        lookahead_point = None
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            dist = math.hypot(px - x, py - y)
            if dist >= lookahead_distance:
                lookahead_point = (px, py)
                break
        
        if lookahead_point is None:
            lookahead_point = path[-1]
        
        # 로봇 좌표계로 변환
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        
        # 로봇 기준 lookahead point
        local_x = math.cos(theta) * dx + math.sin(theta) * dy
        local_y = -math.sin(theta) * dx + math.cos(theta) * dy
        
        # 곡률 계산
        curvature = 2.0 * local_y / (lookahead_distance ** 2)
        
        # Steering angle 계산 (Ackermann)
        steering_angle = math.atan(curvature * self.wheelbase)
        
        # Steering angle 제한
        max_steering = math.radians(30)
        steering_angle = max(-max_steering, min(max_steering, steering_angle))
        
        # Angular velocity 계산 (Differential)
        angular_z = target_speed * curvature
        
        # Ackermann 메시지 생성
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.frame_id = 'base_link'
        
        # ✅ 올바른 stamp 설정
        now = node.get_clock().now()
        ackermann_msg.header.stamp = Time(sec=now.seconds_nanoseconds()[0], 
                                           nanosec=now.seconds_nanoseconds()[1])
        
        ackermann_msg.drive.speed = float(target_speed)
        ackermann_msg.drive.steering_angle = float(steering_angle)
        
        return target_speed, angular_z, ackermann_msg
