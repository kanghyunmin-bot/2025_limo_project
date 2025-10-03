#!/usr/bin/env python3
"""
경로 추종 제어 모듈
- 경로 추종 알고리즘
- 제어 명령 계산
"""
import math
import numpy as np

from path_follower_pkg.math_utils import wrap_to_pi


class PathController:
    """경로 추종 제어기 클래스"""
    
    def __init__(self, k_y=2.0, k_psi=1.8, k_ld=1.0, L_min=0.25, L_max=0.9,
                 v_max=0.8, a_long=0.8, omega_max=3.5):
        self.k_y = k_y
        self.k_psi = k_psi
        self.k_ld = k_ld
        self.L_min = L_min
        self.L_max = L_max
        self.v_max = v_max
        self.a_long = a_long
        self.omega_max = omega_max

    def compute_control(self, x, y, yaw, path_xy, path_v, path_kappa, 
                       idx, ds, s_done, path_len):
        """경로 추종 제어 명령 계산"""
        if idx >= len(path_xy) - 2:
            return 0.0, 0.0

        k_next = idx + 1
        dx, dy = (path_xy[k_next] - path_xy[idx])
        path_yaw = math.atan2(dy, dx)
        e_psi = wrap_to_pi(path_yaw - yaw)

        R = np.array([
            [math.cos(yaw), math.sin(yaw)],
            [-math.sin(yaw), math.cos(yaw)]
        ])
        P = np.array([x, y])
        delta = R @ (path_xy[idx] - P)
        e_y = float(delta[1])

        v_ff = float(path_v[idx])
        Ld = float(np.clip(self.k_ld * v_ff, self.L_min, self.L_max))
        
        s_target = idx * ds + Ld
        idx_target = int(np.clip(round(s_target / ds), 0, len(path_xy) - 1))
        Pt = path_xy[idx_target]
        delta_t = R @ (Pt - P)
        
        if (delta_t[0]**2 + delta_t[1]**2) < 1e-9:
            kappa_pp = 0.0
        else:
            kappa_pp = 2.0 * delta_t[1] / (delta_t[0]**2 + delta_t[1]**2)

        kappa_ref = float(path_kappa[idx])
        omega_ff = v_ff * kappa_ref
        omega_fb = self.k_y * e_y + self.k_psi * math.sin(e_psi)
        omega_pp = v_ff * kappa_pp
        omega = omega_ff + 0.6 * omega_fb + 0.4 * omega_pp
        omega = float(np.clip(omega, -self.omega_max, self.omega_max))

        s_remain = max(0.0, path_len - s_done)
        v_goal = min(v_ff, math.sqrt(2 * self.a_long * max(0.0, s_remain)))
        v_cmd = float(np.clip(v_goal, 0.0, self.v_max))

        return v_cmd, omega

    def check_goal_reached(self, x, y, yaw, goal_xy, goal_yaw, 
                          tol_xy=0.05, tol_yaw=0.09):
        """목표 도달 여부 확인"""
        dist = np.linalg.norm(goal_xy - np.array([x, y]))
        yaw_error = abs(wrap_to_pi(goal_yaw - yaw))
        return dist < tol_xy and yaw_error < tol_yaw
