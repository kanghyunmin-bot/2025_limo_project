#!/usr/bin/env python3
import numpy as np
import math

def de_casteljau(control_points, t):
    """
    De Casteljau 알고리즘으로 Bézier 곡선의 한 점 계산
    
    Args:
        control_points: [[x0,y0], [x1,y1], [x2,y2], [x3,y3]]
        t: 매개변수 [0, 1]
    
    Returns:
        [x, y]
    """
    points = np.array(control_points, dtype=float)
    n = len(points)
    
    for r in range(1, n):
        for i in range(n - r):
            points[i] = (1 - t) * points[i] + t * points[i + 1]
    
    return points[0]

def subdivide_bezier_curve(control_points, t_start, t_end, num_samples=20):
    """
    Bézier 곡선을 De Casteljau로 분할
    
    Args:
        control_points: [[x0,y0], [x1,y1], [x2,y2], [x3,y3]]
        t_start: 시작 매개변수 [0, 1]
        t_end: 끝 매개변수 [0, 1]
        num_samples: 샘플 개수
    
    Returns:
        분할된 경로 points [[x,y], ...]
    """
    segment = []
    t_values = np.linspace(t_start, t_end, num_samples)
    
    for t in t_values:
        point = de_casteljau(control_points, t)
        segment.append(point)
    
    return np.array(segment)

def find_bezier_parameter_at_distance(control_points, target_distance, max_distance, num_samples=100):
    """
    누적 거리 기준으로 Bézier 매개변수 t 찾기
    
    Args:
        control_points: Bézier control points
        target_distance: 목표 누적 거리
        max_distance: 전체 경로 길이
        num_samples: 샘플 개수
    
    Returns:
        t 매개변수 [0, 1]
    """
    t_values = np.linspace(0, 1, num_samples)
    accumulated_dist = 0.0
    prev_point = de_casteljau(control_points, 0)
    
    for i, t in enumerate(t_values[1:], 1):
        current_point = de_casteljau(control_points, t)
        segment_dist = np.linalg.norm(current_point - prev_point)
        accumulated_dist += segment_dist
        
        if accumulated_dist >= target_distance:
            # 선형 보간으로 정확한 t 찾기
            excess = accumulated_dist - target_distance
            ratio = excess / (segment_dist + 1e-6)
            t_interpolated = t_values[i] - ratio * (t_values[i] - t_values[i-1])
            return np.clip(t_interpolated, 0.0, 1.0)
        
        prev_point = current_point
    
    return 1.0  # 끝에 도달

def extract_local_path_from_global(global_path_poses, robot_pos, lookahead_distance=2.0, num_samples=30):
    """
    Global path에서 로봇 위치 기준으로 local path 추출 (De Casteljau)
    
    Args:
        global_path_poses: Global path (list of PoseStamped)
        robot_pos: [x, y] 로봇 위치
        lookahead_distance: 추출할 경로 길이 (m)
        num_samples: 샘플 개수
    
    Returns:
        local_path_points: [[x, y], ...], orientations: [yaw, ...]
    """
    if len(global_path_poses) < 4:
        # Bézier 불가능하면 그냥 선형 반환
        points = []
        yaws = []
        for pose in global_path_poses:
            points.append([pose.pose.position.x, pose.pose.position.y])
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            yaws.append(yaw)
        return np.array(points), yaws
    
    # 1. 가장 가까운 점 찾기
    min_dist = float('inf')
    nearest_idx = 0
    
    for i, pose in enumerate(global_path_poses):
        px = pose.pose.position.x
        py = pose.pose.position.y
        dist = math.hypot(px - robot_pos[0], py - robot_pos[1])
        if dist < min_dist:
            min_dist = dist
            nearest_idx = i
    
    # 2. Lookahead 끝점 찾기
    accumulated = 0.0
    end_idx = nearest_idx
    
    for i in range(nearest_idx, len(global_path_poses) - 1):
        p1 = global_path_poses[i].pose.position
        p2 = global_path_poses[i+1].pose.position
        seg_len = math.hypot(p2.x - p1.x, p2.y - p1.y)
        accumulated += seg_len
        if accumulated >= lookahead_distance:
            end_idx = i + 1
            break
    
    end_idx = min(end_idx, len(global_path_poses) - 1)
    
    # 3. 해당 구간의 Control Points 추출 (4개씩)
    # Cubic Bézier: p0(nearest), p1, p2, p3(end)
    segment_indices = [
        nearest_idx,
        nearest_idx + (end_idx - nearest_idx) // 3,
        nearest_idx + 2 * (end_idx - nearest_idx) // 3,
        end_idx
    ]
    
    control_points = []
    for idx in segment_indices:
        idx = min(idx, len(global_path_poses) - 1)
        pose = global_path_poses[idx]
        control_points.append([pose.pose.position.x, pose.pose.position.y])
    
    # 4. De Casteljau로 분할
    local_points = subdivide_bezier_curve(control_points, t_start=0.0, t_end=1.0, num_samples=num_samples)
    
    # 5. Orientation 계산
    orientations = []
    for i in range(len(local_points)):
        if i == len(local_points) - 1:
            yaw = orientations[-1] if orientations else 0.0
        else:
            dx = local_points[i+1][0] - local_points[i][0]
            dy = local_points[i+1][1] - local_points[i][1]
            yaw = math.atan2(dy, dx)
        orientations.append(yaw)
    
    return local_points, orientations
