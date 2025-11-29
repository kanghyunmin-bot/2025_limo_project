#!/usr/bin/env python3
import math

import numpy as np

from .path_utils import find_nearest_point_on_path, normalize_angle, quaternion_to_yaw


class StanleyController:
    def __init__(self, k_e=3.5, wheelbase=0.4):
        self.k_e = k_e
        self.wheelbase = wheelbase
        self.last_nearest_idx = 0
        self.initialized = False
    
    def compute_control(self, robot_pose, path_points, velocity_ref, node):
        if len(path_points) < 2:
            return 0.0, 0.0, 0.0
        
        robot_x, robot_y, robot_yaw = robot_pose
        
        robot_pos = [robot_x, robot_y]
        nearest_idx, _ = find_nearest_point_on_path(
            robot_pos,
            robot_yaw,
            path_points,
            last_idx=self.last_nearest_idx,
            window_size=50,
        )
        self.initialized = True
        self.last_nearest_idx = nearest_idx
        
        target = path_points[nearest_idx]
        tx = target.pose.position.x
        ty = target.pose.position.y
        tyaw = quaternion_to_yaw(target.pose.orientation)
        
        heading_error = normalize_angle(tyaw - robot_yaw)
        
        dx = tx - robot_x
        dy = ty - robot_y
        cross_track_error = math.cos(tyaw) * dy - math.sin(tyaw) * dx
        
        drive_mode = getattr(node, 'drive_mode', 'differential')
        
        if drive_mode == 'differential':
            # ✅ 완화된 제자리 회전 조건
            heading_threshold = math.radians(25)  # 15° → 25° (더 관대하게)
            
            # ✅ 속도에 따른 동적 threshold
            speed_factor = max(abs(velocity_ref), 0.1)
            adjusted_threshold = heading_threshold * (0.5 / speed_factor)  # 속도 빠르면 threshold 높음
            
            if abs(heading_error) > adjusted_threshold and abs(cross_track_error) > 0.03:
                # 제자리 회전 (방향 오차 크고 + 경로 많이 벗어남)
                linear_v = 0.0
                angular_z = 1.2 * heading_error  # 1.5 → 1.2 (약하게)
                angular_z = np.clip(angular_z, -2.0, 2.0)
                steering = 0.0
            else:
                # 전진하면서 조정
                linear_v = velocity_ref
                
                v_safe = max(abs(velocity_ref), 0.15)
                steering = heading_error + math.atan2(self.k_e * cross_track_error, v_safe)
                steering = np.clip(steering, -math.radians(30), math.radians(30))
                
                curvature = math.tan(steering) / self.wheelbase
                angular_z = velocity_ref * curvature + 0.3 * heading_error
                angular_z = np.clip(angular_z, -2.0, 2.0)
        
        else:
            # Ackermann
            linear_v = velocity_ref
            
            v_safe = max(abs(velocity_ref), 0.15)
            steering = heading_error + math.atan2(self.k_e * cross_track_error, v_safe)
            steering = np.clip(steering, -math.radians(30), math.radians(30))
            
            angular_z = (2.0 * velocity_ref * math.sin(steering)) / self.wheelbase
            angular_z = np.clip(angular_z, -2.0, 2.0)
        
        return linear_v, angular_z, steering
    
    def reset(self):
        self.last_nearest_idx = 0
        self.initialized = False


class StanleyFeedforwardController(StanleyController):
    """
    Stanley + Feedforward 조합: δ = δ_ff + δ_st

    - δ_ff: 곡률 기반 feedforward = atan(L * κ)
    - δ_st: 기본 Stanley 보정 = heading_error + atan2(k_e * e_y, v_safe)
    """

    def _estimate_curvature(self, path_points, idx):
        p_prev = path_points[max(idx - 1, 0)].pose.position
        p_curr = path_points[idx].pose.position
        p_next = path_points[min(idx + 1, len(path_points) - 1)].pose.position

        a = np.array([p_prev.x, p_prev.y], dtype=float)
        b = np.array([p_curr.x, p_curr.y], dtype=float)
        c = np.array([p_next.x, p_next.y], dtype=float)

        ab = b - a
        bc = c - b
        ac = c - a

        denom = np.linalg.norm(ab) * np.linalg.norm(bc) * np.linalg.norm(ac)
        if denom < 1e-9:
            return 0.0

        area = 0.5 * abs(np.cross(ab, ac))
        curvature = 4.0 * area / (denom + 1e-9)
        return curvature

    def compute_control(self, robot_pose, path_points, velocity_ref, node):
        if len(path_points) < 2:
            return 0.0, 0.0, 0.0

        robot_x, robot_y, robot_yaw = robot_pose

        robot_pos = [robot_x, robot_y]
        nearest_idx, _ = find_nearest_point_on_path(
            robot_pos,
            robot_yaw,
            path_points,
            last_idx=self.last_nearest_idx,
            window_size=50,
        )
        self.initialized = True
        self.last_nearest_idx = nearest_idx

        target = path_points[nearest_idx]
        tx = target.pose.position.x
        ty = target.pose.position.y
        tyaw = quaternion_to_yaw(target.pose.orientation)

        heading_error = normalize_angle(tyaw - robot_yaw)

        dx = tx - robot_x
        dy = ty - robot_y
        cross_track_error = math.cos(tyaw) * dy - math.sin(tyaw) * dx

        curvature_ff = self._estimate_curvature(path_points, nearest_idx)
        delta_ff = math.atan(self.wheelbase * curvature_ff)

        drive_mode = getattr(node, 'drive_mode', 'differential')

        if drive_mode == 'differential':
            heading_threshold = math.radians(25)
            speed_factor = max(abs(velocity_ref), 0.1)
            adjusted_threshold = heading_threshold * (0.5 / speed_factor)

            if abs(heading_error) > adjusted_threshold and abs(cross_track_error) > 0.03:
                linear_v = 0.0
                angular_z = 1.2 * heading_error
                angular_z = np.clip(angular_z, -2.0, 2.0)
                steering = 0.0
            else:
                linear_v = velocity_ref

                v_safe = max(abs(velocity_ref), 0.15)
                delta_st = heading_error + math.atan2(self.k_e * cross_track_error, v_safe)
                steering = delta_ff + delta_st
                steering = np.clip(steering, -math.radians(30), math.radians(30))

                curvature = math.tan(steering) / self.wheelbase
                angular_z = velocity_ref * curvature + 0.3 * heading_error
                angular_z = np.clip(angular_z, -2.0, 2.0)

        else:
            linear_v = velocity_ref
            v_safe = max(abs(velocity_ref), 0.15)
            delta_st = heading_error + math.atan2(self.k_e * cross_track_error, v_safe)
            steering = delta_ff + delta_st
            steering = np.clip(steering, -math.radians(30), math.radians(30))

            angular_z = (2.0 * velocity_ref * math.sin(steering)) / self.wheelbase
            angular_z = np.clip(angular_z, -2.0, 2.0)

        return linear_v, angular_z, steering
