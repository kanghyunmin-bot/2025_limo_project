#!/usr/bin/env python3
"""
곡률 기반 가변 속도 프로파일 생성
직선: 가속, 커브: 감속
"""
import numpy as np
from nav_msgs.msg import Path


class VelocityProfileGenerator:
    """곡률 기반 속도 프로파일 생성기"""
    
    def __init__(self, 
                 v_max_straight=0.5,    # 직선 최대 속도 (m/s)
                 v_min_curve=0.15,      # 코너 최소 속도 (m/s)
                 curvature_threshold=0.5,  # 곡률 임계값 (1/m)
                 robot_width=0.2,       # 로봇 폭 (m)
                 robot_length=0.4,      # 로봇 길이 (m)
                 min_turn_radius=0.5):  # 최소 회전 반경 (m)
        
        self.v_max = v_max_straight
        self.v_min = v_min_curve
        self.k_thresh = curvature_threshold
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.min_radius = min_turn_radius
        
    def compute_curvature(self, path: Path):
        """
        경로의 곡률 계산 (3점 원 피팅 방식)
        
        Args:
            path: nav_msgs/Path
            
        Returns:
            curvatures: numpy array of curvatures (1/m)
        """
        if len(path.poses) < 3:
            return np.zeros(len(path.poses))
        
        curvatures = []
        poses = path.poses
        
        for i in range(len(poses)):
            if i == 0 or i == len(poses) - 1:
                curvatures.append(0.0)
                continue
            
            # 이전, 현재, 다음 점
            p_prev = poses[i-1].pose.position
            p_curr = poses[i].pose.position
            p_next = poses[i+1].pose.position
            
            # 벡터 계산
            v1 = np.array([p_curr.x - p_prev.x, p_curr.y - p_prev.y])
            v2 = np.array([p_next.x - p_curr.x, p_next.y - p_curr.y])
            
            # 거리
            d1 = np.linalg.norm(v1)
            d2 = np.linalg.norm(v2)
            
            if d1 < 1e-6 or d2 < 1e-6:
                curvatures.append(0.0)
                continue
            
            # 각도 변화
            cos_theta = np.dot(v1, v2) / (d1 * d2)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            theta = np.arccos(cos_theta)
            
            # 곡률 = 각도 변화 / 평균 거리
            # Menger curvature 근사
            curvature = 2.0 * np.sin(theta / 2.0) / ((d1 + d2) / 2.0)
            curvatures.append(abs(curvature))
        
        return np.array(curvatures)
    
    def generate_velocity_profile(self, path: Path):
        """
        곡률에 따른 속도 프로파일 생성
        
        Args:
            path: nav_msgs/Path
            
        Returns:
            velocities: numpy array of velocities (m/s)
        """
        curvatures = self.compute_curvature(path)
        velocities = []
        
        for k in curvatures:
            # 곡률에 따른 속도 계산
            if k > 1e-6:
                radius = 1.0 / k
                
                # 최소 회전 반경 체크
                if radius < self.min_radius:
                    # 급커브: 최소 속도
                    v = self.v_min
                else:
                    # 완만한 커브: 선형 보간
                    # k가 클수록 (반경이 작을수록) 느리게
                    ratio = min(k / self.k_thresh, 1.0)
                    v = self.v_max - (self.v_max - self.v_min) * ratio
            else:
                # 직선: 최대 속도
                v = self.v_max
            
            velocities.append(v)
        
        return np.array(velocities)
    
    def get_velocity_at_index(self, velocities, index):
        """
        특정 인덱스의 속도 반환 (경계 체크)
        
        Args:
            velocities: numpy array
            index: int
            
        Returns:
            velocity: float (m/s)
        """
        if velocities is None or len(velocities) == 0:
            return self.v_min
        
        if index < 0:
            return float(velocities[0])
        if index >= len(velocities):
            return float(velocities[-1])
        
        return float(velocities[index])
    
    def update_params(self, v_max=None, v_min=None, min_radius=None):
        """파라미터 동적 업데이트 (GUI용)"""
        if v_max is not None:
            self.v_max = v_max
        if v_min is not None:
            self.v_min = v_min
        if min_radius is not None:
            self.min_radius = min_radius
