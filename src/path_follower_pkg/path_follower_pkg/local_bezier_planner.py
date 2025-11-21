#!/usr/bin/env python3
import numpy as np
from scipy.optimize import minimize

class LocalBezierPlanner:
    def __init__(self, 
                 lookahead_dist=0.4,
                 angle_constraint=60.0,
                 r_min=0.2, r_max=0.4,
                 alpha=1.0, beta=2.0, gamma=5.0, delta=3.0,
                 # ✅ 드 카스텔조 파라미터
                 de_casteljau_iterations=20,      # 이진 탐색 반복 횟수
                 de_casteljau_samples=10,         # 거리 계산용 샘플 수
                 de_casteljau_tolerance=0.01):    # 수렴 허용 오차 (m)
        
        # 기본 파라미터
        self.lookahead_dist = lookahead_dist
        self.angle_constraint = np.radians(angle_constraint)
        self.r_min = r_min
        self.r_max = r_max
        
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        
        # ✅ 드 카스텔조 설정
        self.de_casteljau_iterations = de_casteljau_iterations
        self.de_casteljau_samples = de_casteljau_samples
        self.de_casteljau_tolerance = de_casteljau_tolerance
        
        self.prev_p1 = None
        self.prev_p2 = None
        self.global_waypoints = None
        
    def set_global_path(self, waypoints):
        self.global_waypoints = np.array(waypoints)
    
    def de_casteljau(self, control_points, t):
        """
        ✅ 드 카스텔조 알고리즘 (De Casteljau's Algorithm)
        
        Args:
            control_points: 제어점 리스트 [p0, p1, p2, p3]
            t: 파라미터 [0, 1]
        
        Returns:
            베지어 곡선 상의 점
        """
        points = np.array(control_points)
        n = len(points)
        
        for r in range(1, n):
            for i in range(n - r):
                points[i] = (1 - t) * points[i] + t * points[i + 1]
        
        return points[0]
    
    def get_point_at_distance_on_segment(self, p0, p1, p2, p3, target_dist):
        """
        ✅ 베지어 구간에서 target_dist 떨어진 점 찾기
        
        Args:
            p0, p1, p2, p3: 큐빅 베지어 제어점
            target_dist: 목표 거리 (m)
        
        Returns:
            (point, remaining_dist)
        """
        # ✅ 이진 탐색으로 t 찾기
        t_low, t_high = 0.0, 1.0
        
        for _ in range(self.de_casteljau_iterations):
            t_mid = (t_low + t_high) / 2.0
            
            # t=0 부터 t_mid까지 거리 계산
            dist = 0.0
            prev_point = p0
            
            for i in range(1, self.de_casteljau_samples + 1):
                t = (i / self.de_casteljau_samples) * t_mid
                curr_point = self.de_casteljau([p0, p1, p2, p3], t)
                dist += np.linalg.norm(curr_point - prev_point)
                prev_point = curr_point
            
            # ✅ 수렴 체크
            if abs(dist - target_dist) < self.de_casteljau_tolerance:
                point = self.de_casteljau([p0, p1, p2, p3], t_mid)
                return point, 0.0
            elif dist < target_dist:
                t_low = t_mid
            else:
                t_high = t_mid
        
        # 끝까지 도달
        point = self.de_casteljau([p0, p1, p2, p3], 1.0)
        
        # 구간 전체 길이 계산
        total_dist = 0.0
        prev_point = p0
        for i in range(1, self.de_casteljau_samples + 1):
            t = i / self.de_casteljau_samples
            curr_point = self.de_casteljau([p0, p1, p2, p3], t)
            total_dist += np.linalg.norm(curr_point - prev_point)
            prev_point = curr_point
        
        remaining = target_dist - total_dist
        return point, max(0, remaining)
    
    def get_point_on_global_path(self, robot_pos, distance):
        """Global path에서 distance 떨어진 점"""
        if self.global_waypoints is None or len(self.global_waypoints) < 2:
            return robot_pos + np.array([distance, 0])
        
        # 가장 가까운 구간 찾기
        min_dist = float('inf')
        closest_idx = 0
        
        for i in range(len(self.global_waypoints) - 1):
            p1 = self.global_waypoints[i]
            p2 = self.global_waypoints[i + 1]
            
            v = p2 - p1
            w = robot_pos - p1
            
            c1 = np.dot(w, v)
            if c1 <= 0:
                dist = np.linalg.norm(robot_pos - p1)
            else:
                c2 = np.dot(v, v)
                if c1 >= c2:
                    dist = np.linalg.norm(robot_pos - p2)
                else:
                    b = c1 / c2
                    pb = p1 + b * v
                    dist = np.linalg.norm(robot_pos - pb)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        remaining_dist = distance
        
        for i in range(closest_idx, len(self.global_waypoints) - 1):
            p1 = self.global_waypoints[i]
            p2 = self.global_waypoints[i + 1]
            
            seg_len = np.linalg.norm(p2 - p1)
            
            if remaining_dist <= seg_len:
                t = remaining_dist / seg_len if seg_len > 0 else 0
                return p1 + t * (p2 - p1)
            
            remaining_dist -= seg_len
        
        return self.global_waypoints[-1]
    
    def cubic_bezier(self, p0, p1, p2, p3, num_samples=10):
        """큐빅 베지어 곡선 생성"""
        t = np.linspace(0, 1, num_samples)
        curve = np.outer((1-t)**3, p0) + \
                np.outer(3*(1-t)**2*t, p1) + \
                np.outer(3*(1-t)*t**2, p2) + \
                np.outer(t**3, p3)
        return curve
    
    def get_default_control_points(self, p0, p3):
        direction = p3 - p0
        dist = np.linalg.norm(direction)
        
        if dist < 1e-6:
            return p0 + np.array([0.1, 0]), p0 + np.array([0.2, 0])
        
        unit_dir = direction / dist
        p1_default = p0 + unit_dir * (dist * 0.33)
        p2_default = p0 + unit_dir * (dist * 0.67)
        
        return p1_default, p2_default
    
    def cost_function(self, x, p0, p3, p1_default, p2_default, obstacles):
        p1 = np.array([x[0], x[1]])
        p2 = np.array([x[2], x[3]])
        
        curve = self.cubic_bezier(p0, p1, p2, p3, num_samples=10)
        
        length = np.sum(np.linalg.norm(np.diff(curve, axis=0), axis=1))
        
        curvature_cost = 0.0
        for i in range(1, len(curve) - 1):
            v1 = curve[i] - curve[i-1]
            v2 = curve[i+1] - curve[i]
            
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            
            if norm1 > 1e-6 and norm2 > 1e-6:
                cos_angle = np.dot(v1, v2) / (norm1 * norm2)
                cos_angle = np.clip(cos_angle, -1.0, 1.0)
                angle_change = np.arccos(cos_angle)
                curvature_cost += angle_change ** 2
        
        obstacle_cost = 0.0
        if len(obstacles) > 0:
            for obs_x, obs_y, obs_r in obstacles:
                obs_pos = np.array([obs_x, obs_y])
                dists = np.linalg.norm(curve - obs_pos, axis=1)
                min_dist = np.min(dists)
                
                if min_dist < obs_r + 0.2:
                    obstacle_cost += 100.0 * (obs_r + 0.2 - min_dist) ** 2
        
        deviation_cost = np.linalg.norm(p1 - p1_default) ** 2 + \
                         np.linalg.norm(p2 - p2_default) ** 2
        
        total_cost = (self.alpha * length +
                      self.beta * curvature_cost +
                      self.gamma * obstacle_cost +
                      self.delta * deviation_cost)
        
        return total_cost
    
    def plan_local_path(self, robot_pos, robot_yaw, obstacles=[]):
        p0 = np.array(robot_pos)
        p3 = self.get_point_on_global_path(p0, self.lookahead_dist)
        
        p1_default, p2_default = self.get_default_control_points(p0, p3)
        
        if self.prev_p1 is not None:
            x0 = np.concatenate([self.prev_p1, self.prev_p2])
        else:
            x0 = np.concatenate([p1_default, p2_default])
        
        bounds = [
            (p0[0] - self.r_max, p0[0] + self.r_max),
            (p0[1] - self.r_max, p0[1] + self.r_max),
            (p0[0] - self.r_max, p0[0] + self.r_max),
            (p0[1] - self.r_max, p0[1] + self.r_max),
        ]
        
        result = minimize(
            self.cost_function,
            x0,
            args=(p0, p3, p1_default, p2_default, obstacles),
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 20, 'ftol': 1e-4}
        )
        
        p1_opt = result.x[:2]
        p2_opt = result.x[2:]
        
        self.prev_p1 = p1_opt
        self.prev_p2 = p2_opt
        
        curve = self.cubic_bezier(p0, p1_opt, p2_opt, p3, num_samples=15)
        
        return curve
