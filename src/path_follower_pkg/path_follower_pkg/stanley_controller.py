#!/usr/bin/env python3
import math
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time


class StanleyController:
    def __init__(self, k_e=1.0, k_h=0.5, wheelbase=0.4):
        self.k_e = k_e
        self.k_h = k_h
        self.wheelbase = wheelbase
    
    def compute_control(self, robot_pose, path, target_speed, node):
        """
        Stanley Method 제어
        robot_pose: [x, y, theta]
        path: [[x1, y1], [x2, y2], ...]
        target_speed: 목표 속도
        node: ROS2 노드 (시간 정보용)
        """
        if len(path) < 2:
            return 0.0, 0.0, None
        
        x, y, theta = robot_pose
        
        # 1. 가장 가까운 경로점 찾기
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(path):
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 2. 경로 탄젠트 방향 계산
        if closest_idx < len(path) - 1:
            p1 = path[closest_idx]
            p2 = path[closest_idx + 1]
        else:
            p1 = path[-2]
            p2 = path[-1]
        
        path_angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        
        # 3. 헤딩 오차
        heading_error = path_angle - theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # 4. 횡방향 오차 (Cross Track Error)
        dx = path[closest_idx][0] - x
        dy = path[closest_idx][1] - y
        
        # 로봇 좌표계로 변환
        cross_track_error = -math.sin(theta) * dx + math.cos(theta) * dy
        
        # 5. Stanley 제어식
        # δ = heading_error + atan(k_e * CTE / v)
        v = max(target_speed, 0.1)  # 0으로 나누기 방지
        steering_angle = heading_error + math.atan2(self.k_e * cross_track_error, v)
        
        # Steering angle 제한 (-30도 ~ 30도)
        max_steering = math.radians(30)
        steering_angle = max(-max_steering, min(max_steering, steering_angle))
        
        # 6. Ackermann 메시지 생성
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.frame_id = 'base_link'
        
        # ✅ 올바른 stamp 설정
        now = node.get_clock().now()
        ackermann_msg.header.stamp = Time(sec=now.seconds_nanoseconds()[0], 
                                           nanosec=now.seconds_nanoseconds()[1])
        
        ackermann_msg.drive.speed = float(target_speed)
        ackermann_msg.drive.steering_angle = float(steering_angle)
        
        # 7. Differential용 angular.z 계산
        if abs(steering_angle) > 0.001:
            angular_z = target_speed * math.tan(steering_angle) / self.wheelbase
        else:
            angular_z = 0.0
        
        return target_speed, angular_z, ackermann_msg
