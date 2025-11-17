#!/usr/bin/env python3
"""
Planner Interface Module
알고리즘팀에서 보내는 /planner/path를 수신하고 처리하는 전용 모듈
"""

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np


class PlannerInterface:
    """
    알고리즘팀 Planner Path 인터페이스
    
    역할:
    - /planner/path 토픽 구독
    - Path 메시지 파싱 및 검증
    - waypoint 리스트 추출
    - 경로 정보 (거리, 곡률 등) 계산
    """
    
    def __init__(self, node):
        """
        Parameters:
        - node: ROS2 Node 인스턴스
        """
        self.node = node
        self.current_path = None
        self.waypoints = []
        self.path_valid = False
        
        self.node.get_logger().info("📦 Planner Interface initialized")
    
    def validate_path(self, path_msg: Path):
        """
        Planner Path 검증
        
        Returns:
        - True: 유효한 경로
        - False: 무효한 경로
        """
        if path_msg is None:
            return False
        
        if len(path_msg.poses) < 2:
            self.node.get_logger().warn("⚠️ Path too short (< 2 poses)")
            return False
        
        # 경로 간격 체크 (너무 촘촘하거나 너무 멀면 경고)
        for i in range(len(path_msg.poses) - 1):
            p1 = path_msg.poses[i].pose.position
            p2 = path_msg.poses[i+1].pose.position
            dist = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            
            if dist > 10.0:  # 10m 이상 간격이면 경고
                self.node.get_logger().warn(f"⚠️ Large gap detected: {dist:.2f}m")
        
        return True
    
    def process_path(self, path_msg: Path):
        """
        Planner Path 처리
        
        Returns:
        - waypoints: [(x, y), ...] 리스트
        - path_msg: 원본 Path 메시지
        """
        if not self.validate_path(path_msg):
            return None, None
        
        # Waypoint 추출
        self.waypoints = []
        for pose in path_msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.waypoints.append([x, y])
        
        self.current_path = path_msg
        self.path_valid = True
        
        self.node.get_logger().info(
            f"✓ Planner Path processed: {len(self.waypoints)} waypoints"
        )
        
        return self.waypoints, path_msg
    
    def get_waypoints(self):
        """현재 waypoints 반환"""
        return self.waypoints
    
    def get_path(self):
        """현재 Path 메시지 반환"""
        return self.current_path
    
    def is_valid(self):
        """경로 유효성 확인"""
        return self.path_valid
    
    def reset(self):
        """경로 초기화"""
        self.current_path = None
        self.waypoints = []
        self.path_valid = False
        self.node.get_logger().info("🔄 Planner Interface reset")
    
    def calculate_path_info(self):
        """
        경로 정보 계산
        
        Returns:
        - distance: 총 경로 길이 (m)
        - max_curvature: 최대 곡률
        """
        if not self.path_valid or len(self.waypoints) < 2:
            return 0.0, 0.0
        
        # 총 거리 계산
        distance = 0.0
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i+1]
            distance += np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        
        # 최대 곡률 계산 (3점 기반)
        curvatures = []
        for i in range(1, len(self.waypoints) - 1):
            p0 = np.array(self.waypoints[i-1])
            p1 = np.array(self.waypoints[i])
            p2 = np.array(self.waypoints[i+1])
            
            v1 = p1 - p0
            v2 = p2 - p1
            
            len1 = np.linalg.norm(v1)
            len2 = np.linalg.norm(v2)
            
            if len1 > 0 and len2 > 0:
                cross = v1[0]*v2[1] - v1[1]*v2[0]
                curvature = abs(cross) / (len1 * len2)
                curvatures.append(curvature)
        
        max_curvature = max(curvatures) if curvatures else 0.0
        
        return distance, max_curvature
