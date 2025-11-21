#!/usr/bin/env python3
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class PathRecorder:
    """실제 주행 경로 기록"""
    
    def __init__(self, record_interval: float = 0.1):
        """
        Args:
            record_interval: 기록 간격 (미터), 기본 10cm
        """
        self.record_interval = record_interval
        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'
        self.last_recorded_pose = None
    
    def reset(self):
        """경로 초기화"""
        self.actual_path.poses.clear()
        self.last_recorded_pose = None
    
    def record(self, odom_msg: Odometry) -> Path:
        """
        Odometry 메시지로부터 경로 기록
        
        Args:
            odom_msg: Odometry 메시지
        
        Returns:
            actual_path: 업데이트된 실제 경로
        """
        current_pose = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
        
        if self.last_recorded_pose is None:
            # 첫 기록
            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose
            self.actual_path.poses.append(pose_stamped)
            self.last_recorded_pose = current_pose
        else:
            # 일정 거리 이동 시 기록
            dist = math.hypot(
                current_pose[0] - self.last_recorded_pose[0],
                current_pose[1] - self.last_recorded_pose[1]
            )
            if dist >= self.record_interval:
                pose_stamped = PoseStamped()
                pose_stamped.header = odom_msg.header
                pose_stamped.pose = odom_msg.pose.pose
                self.actual_path.poses.append(pose_stamped)
                self.last_recorded_pose = current_pose
        
        return self.actual_path
    
    def get_path(self) -> Path:
        """현재 기록된 경로 반환"""
        return self.actual_path
