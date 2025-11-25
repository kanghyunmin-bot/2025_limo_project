#!/usr/bin/env python3
import numpy as np
import math

def de_casteljau_subdivision(control_points, t_start, t_end):
    """
    De Casteljau 알고리즘으로 Bézier 곡선을 분할하여 새 Control Points 생성
    
    Args:
        control_points: [[x0,y0], [x1,y1], [x2,y2], [x3,y3]] (Cubic Bézier)
        t_start: 시작 매개변수 [0, 1]
        t_end: 끝 매개변수 [0, 1]
    
    Returns:
        new_control_points: 분할된 구간의 새 Control Points [[x0',y0'], [x1',y1'], [x2',y2'], [x3',y3']]
    """
    p = np.array(control_points, dtype=float)
    
    # 1단계: t_start에서 분할 (왼쪽 부분)
    if t_start > 0:
        t = t_start
        # De Casteljau 단계별 계산
        p01 = (1-t)*p[0] + t*p[1]
        p12 = (1-t)*p[1] + t*p[2]
        p23 = (1-t)*p[2] + t*p[3]
        
        p012 = (1-t)*p01 + t*p12
        p123 = (1-t)*p12 + t*p23
        
        p0123 = (1-t)*p012 + t*p123
        
        # 오른쪽 부분의 Control Points
        p = np.array([p0123, p123, p23, p[3]])
    
    # 2단계: (t_end - t_start) / (1 - t_start)로 다시 분할 (왼쪽 부분)
    if t_end < 1:
        # t를 새 매개변수 공간으로 변환
        t = (t_end - t_start) / (1.0 - t_start) if t_start < 1 else 0
        t = np.clip(t, 0, 1)
        
        # De Casteljau 단계별 계산
        p01 = (1-t)*p[0] + t*p[1]
        p12 = (1-t)*p[1] + t*p[2]
        p23 = (1-t)*p[2] + t*p[3]
        
        p012 = (1-t)*p01 + t*p12
        p123 = (1-t)*p12 + t*p23
        
        p0123 = (1-t)*p012 + t*p123
        
        # 왼쪽 부분의 Control Points
        p = np.array([p[0], p01, p012, p0123])
    
    return p.tolist()

def evaluate_bezier(control_points, t):
    """Bézier 곡선의 한 점 계산"""
    p = np.array(control_points, dtype=float)
    n = len(p)
    
    for r in range(1, n):
        for i in range(n - r):
            p[i] = (1 - t) * p[i] + t * p[i + 1]
    
    return p[0]

def subdivide_bezier_with_sampling(control_points, t_start, t_end, num_samples=30):
    """
    De Casteljau로 분할된 Bézier 곡선을 샘플링
    
    Args:
        control_points: 원본 Control Points
        t_start, t_end: 분할 범위
        num_samples: 샘플 개수
    
    Returns:
        sampled_points: [[x,y], ...], orientations: [yaw, ...]
    """
    # 새 Control Points 계산
    new_cp = de_casteljau_subdivision(control_points, t_start, t_end)
    
    # 새 매개변수 공간 [0, 1]에서 샘플링
    t_values = np.linspace(0, 1, num_samples)
    points = []
    
    for t in t_values:
        point = evaluate_bezier(new_cp, t)
        points.append(point)
    
    points = np.array(points)
    
    # Orientation 계산
    orientations = []
    for i in range(len(points)):
        if i == len(points) - 1:
            yaw = orientations[-1] if orientations else 0.0
        else:
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            yaw = math.atan2(dy, dx)
        orientations.append(yaw)
    
    return points, orientations

def extract_local_path_with_cp(global_path_poses, robot_pos, lookahead_distance=2.0, num_samples=30):
    """
    Global path에서 Control Point 기반으로 Local path 추출
    
    Args:
        global_path_poses: Global path (list of PoseStamped)
        robot_pos: [x, y]
        lookahead_distance: 추출 거리
        num_samples: 샘플 개수
    
    Returns:
        points: [[x,y], ...], orientations: [yaw, ...]
    """
    if len(global_path_poses) < 4:
        # Bézier 불가능하면 선형
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
    
    # 3. 전체 경로 길이 계산
    total_length = 0.0
    for i in range(len(global_path_poses) - 1):
        p1 = global_path_poses[i].pose.position
        p2 = global_path_poses[i+1].pose.position
        total_length += math.hypot(p2.x - p1.x, p2.y - p1.y)
    
    # 4. nearest_idx와 end_idx의 누적 거리 → t_start, t_end
    t_start = 0.0
    accumulated = 0.0
    for i in range(nearest_idx):
        if i >= len(global_path_poses) - 1:
            break
        p1 = global_path_poses[i].pose.position
        p2 = global_path_poses[i+1].pose.position
        accumulated += math.hypot(p2.x - p1.x, p2.y - p1.y)
    t_start = accumulated / (total_length + 1e-6)
    
    t_end = 0.0
    accumulated = 0.0
    for i in range(end_idx):
        if i >= len(global_path_poses) - 1:
            break
        p1 = global_path_poses[i].pose.position
        p2 = global_path_poses[i+1].pose.position
        accumulated += math.hypot(p2.x - p1.x, p2.y - p1.y)
    t_end = accumulated / (total_length + 1e-6)
    
    t_start = np.clip(t_start, 0.0, 1.0)
    t_end = np.clip(t_end, 0.0, 1.0)
    
    # 5. 전체 경로를 대표하는 4개 Control Points 추출
    indices = [
        0,
        len(global_path_poses) // 3,
        (len(global_path_poses) * 2) // 3,
        len(global_path_poses) - 1
    ]
    
    control_points = []
    for idx in indices:
        idx = min(idx, len(global_path_poses) - 1)
        pose = global_path_poses[idx]
        control_points.append([pose.pose.position.x, pose.pose.position.y])
    
    # 6. De Casteljau로 분할
    points, orientations = subdivide_bezier_with_sampling(
        control_points, t_start, t_end, num_samples
    )
    
    return points, orientations
