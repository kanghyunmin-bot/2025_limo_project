#!/usr/bin/env python3
import math
from typing import Sequence

import numpy as np

def quaternion_to_yaw(q):
    return 2.0 * math.atan2(q.z, q.w)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def find_nearest_point_on_path(
    robot_pos,
    robot_yaw,
    path_poses: Sequence,
    last_idx: int = 0,
    window_size: int = 50,
    loss_threshold: float = 1.0,
):
    """윈도우 기반 최근접점 탐색 (NumPy 가속, fallback 포함)."""

    if not path_poses:
        return 0, float("inf")

    start = max(0, last_idx - 10)
    end = min(len(path_poses), last_idx + window_size)
    local_path = path_poses[start:end]

    pts = np.array([[p.pose.position.x, p.pose.position.y] for p in local_path], dtype=float)
    robot_xy = np.array(robot_pos[:2], dtype=float)
    dist_sq = np.sum((pts - robot_xy) ** 2, axis=1)
    local_idx = int(np.argmin(dist_sq))
    min_dist = float(math.sqrt(dist_sq[local_idx]))
    nearest_idx = start + local_idx

    if min_dist > loss_threshold:
        full_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in path_poses], dtype=float)
        dist_sq_full = np.sum((full_pts - robot_xy) ** 2, axis=1)
        nearest_idx = int(np.argmin(dist_sq_full))
        min_dist = float(math.sqrt(dist_sq_full[nearest_idx]))

    return nearest_idx, min_dist

def compute_cross_track_error(robot_pos, robot_yaw, path_pose):
    px = path_pose.pose.position.x
    py = path_pose.pose.position.y
    path_yaw = quaternion_to_yaw(path_pose.pose.orientation)
    
    dx = px - robot_pos[0]
    dy = py - robot_pos[1]
    
    cross_track_error = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy
    heading_error = normalize_angle(path_yaw - robot_yaw)
    
    return cross_track_error, heading_error
