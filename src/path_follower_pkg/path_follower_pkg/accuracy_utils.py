#!/usr/bin/env python3
import math
from nav_msgs.msg import Path

class AccuracyCalculator:
    """경로 정확도 계산 유틸"""
    
    @staticmethod
    def calculate_accuracy(actual_path: Path, reference_path: Path) -> float:
        """
        실제 경로와 참조 경로 간 정확도 계산
        
        Args:
            actual_path: 실제 주행 경로
            reference_path: 참조 경로 (계획 경로)
        
        Returns:
            accuracy: 정확도 (0~100%)
        """
        if len(actual_path.poses) < 2:
            return 100.0
        
        if reference_path is None or len(reference_path.poses) < 2:
            return 100.0
        
        total_error = 0.0
        count = 0
        
        for actual_pose in actual_path.poses:
            ax = actual_pose.pose.position.x
            ay = actual_pose.pose.position.y
            
            # 참조 경로에서 가장 가까운 점 찾기
            min_dist = float('inf')
            for ref_pose in reference_path.poses:
                px = ref_pose.pose.position.x
                py = ref_pose.pose.position.y
                dist = math.hypot(ax - px, ay - py)
                if dist < min_dist:
                    min_dist = dist
            
            total_error += min_dist
            count += 1
        
        avg_error = total_error / count if count > 0 else 0.0
        accuracy = max(0.0, 100.0 - (avg_error * 100))
        
        return accuracy
