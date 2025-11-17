#!/usr/bin/env python3
import numpy as np
from scipy.optimize import minimize

class LocalBezierPlanner:
    def __init__(self, 
                 lookahead_dist=1.0,
                 angle_constraint=60.0,
                 r_min=0.3, r_max=0.8,
                 alpha=1.0, beta=2.0, gamma=5.0, delta=3.0):
        self.lookahead_dist = lookahead_dist
        self.angle_constraint = np.radians(angle_constraint)
        self.r_min = r_min
        self.r_max = r_max
        
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
        
        self.prev_p1 = None
        self.prev_p2 = None
        self.global_waypoints = None
        
    def set_global_path(self, waypoints):
        self.global_waypoints = np.array(waypoints)
        
    def cubic_bezier(self, p0, p1, p2, p3, num_samples=15):  # ✅ 20→15 (속도 향상)
        t = np.linspace(0, 1, num_samples)
        curve = np.outer((1-t)**3, p0) + \
                np.outer(3*(1-t)**2*t, p1) + \
                np.outer(3*(1-t)*t**2, p2) + \
                np.outer(t**3, p3)
        return curve
    
    def get_point_on_global_path(self, robot_pos, distance):
        if self.global_waypoints is None or len(self.global_waypoints) < 2:
            return robot_pos + np.array([distance, 0])
        
        dists = np.linalg.norm(self.global_waypoints - robot_pos, axis=1)
        closest_idx = np.argmin(dists)
        
        accumulated = 0.0
        for i in range(closest_idx, len(self.global_waypoints) - 1):
            p1 = self.global_waypoints[i]
            p2 = self.global_waypoints[i+1]
            seg_len = np.linalg.norm(p2 - p1)
            
            if accumulated + seg_len >= distance:
                remaining = distance - accumulated
                t = remaining / seg_len if seg_len > 0 else 0
                return p1 + t * (p2 - p1)
            
            accumulated += seg_len
        
        return self.global_waypoints[-1]
    
    def get_default_control_points(self, p0, p3):
        direction = p3 - p0
        dist = np.linalg.norm(direction)
        
        if dist < 1e-6:
            return p0 + np.array([0.3, 0]), p0 + np.array([0.6, 0])
        
        unit_dir = direction / dist
        p1_default = p0 + unit_dir * (dist * 0.33)
        p2_default = p0 + unit_dir * (dist * 0.67)
        
        return p1_default, p2_default
    
    def cost_function(self, x, p0, p3, p1_default, p2_default, obstacles):
        p1 = np.array([x[0], x[1]])
        p2 = np.array([x[2], x[3]])
        
        curve = self.cubic_bezier(p0, p1, p2, p3, num_samples=15)
        
        # (1) 경로 길이
        length = np.sum(np.linalg.norm(np.diff(curve, axis=0), axis=1))
        
        # (2) 곡률^2
        curvature_cost = 0.0
        for i in range(1, len(curve) - 1):
            v1 = curve[i] - curve[i-1]
            v2 = curve[i+1] - curve[i]
            
            if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                angle_change = np.arccos(np.clip(
                    np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)),
                    -1.0, 1.0
                ))
                curvature_cost += angle_change ** 2
        
        # (3) 장애물 회피
        obstacle_cost = 0.0
        for obs_x, obs_y, obs_r in obstacles:
            obs_pos = np.array([obs_x, obs_y])
            dists = np.linalg.norm(curve - obs_pos, axis=1)
            min_dist = np.min(dists)
            
            if min_dist < obs_r + 0.2:
                obstacle_cost += 100.0 * (obs_r + 0.2 - min_dist) ** 2
        
        # (4) 전역 경로 이탈 페널티
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
        
        # Warm start
        if self.prev_p1 is not None:
            x0 = np.concatenate([self.prev_p1, self.prev_p2])
        else:
            x0 = np.concatenate([p1_default, p2_default])
        
        # Bounds
        bounds = [
            (p0[0] - self.r_max, p0[0] + self.r_max),
            (p0[1] - self.r_max, p0[1] + self.r_max),
            (p0[0] - self.r_max, p0[0] + self.r_max),
            (p0[1] - self.r_max, p0[1] + self.r_max),
        ]
        
        # ✅ 최적화 (50→30 iteration, 속도 향상)
        result = minimize(
            self.cost_function,
            x0,
            args=(p0, p3, p1_default, p2_default, obstacles),
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 30}  # 50→30
        )
        
        p1_opt = result.x[:2]
        p2_opt = result.x[2:]
        
        self.prev_p1 = p1_opt
        self.prev_p2 = p2_opt
        
        curve = self.cubic_bezier(p0, p1_opt, p2_opt, p3, num_samples=20)
        
        return curve
