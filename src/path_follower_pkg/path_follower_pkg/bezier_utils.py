#!/usr/bin/env python3
import numpy as np
import math


def bezier_curve(control_points, num_points=50):
    """
    Bézier curve 생성
    control_points: [P0, P1, P2, P3] (4개 제어점)
    """
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 2))
    
    for i, t in enumerate(t_values):
        point = np.zeros(2)
        for j in range(n + 1):
            # Bernstein polynomial
            binomial = math.comb(n, j)
            bernstein = binomial * (t ** j) * ((1 - t) ** (n - j))
            point += bernstein * control_points[j]
        curve[i] = point
    
    return curve


def split_global_to_local_bezier(global_path, robot_pos, lookahead_dist=1.0):
    """
    ✅ 진짜 Local Bézier curve 생성
    1. Global path에서 로봇 주변 구간 추출
    2. 제어점으로 Bézier curve 생성
    """
    if global_path is None or len(global_path.poses) < 2:
        return None
    
    # Global path를 numpy array로 변환
    global_points = np.array([
        [pose.pose.position.x, pose.pose.position.y]
        for pose in global_path.poses
    ])
    
    # 1. 로봇에서 가장 가까운 점 찾기
    distances = np.linalg.norm(global_points - robot_pos, axis=1)
    nearest_idx = np.argmin(distances)
    
    # 2. Lookahead distance 내의 점들 추출
    segment_points = []
    cumulative_dist = 0.0
    
    for i in range(nearest_idx, len(global_points)):
        segment_points.append(global_points[i])
        
        if i > nearest_idx:
            segment_dist = np.linalg.norm(global_points[i] - global_points[i-1])
            cumulative_dist += segment_dist
            
            if cumulative_dist >= lookahead_dist:
                break
    
    if len(segment_points) < 2:
        return None
    
    segment_points = np.array(segment_points)
    
    # 3. ✅ 제어점 생성 (Cubic Bézier)
    # P0: 시작점
    # P1: 시작점에서 앞쪽으로 1/3
    # P2: 끝점에서 뒤쪽으로 1/3  
    # P3: 끝점
    
    P0 = segment_points[0]
    P3 = segment_points[-1]
    
    # 중간 제어점 계산
    if len(segment_points) >= 4:
        # 충분한 점이 있으면 중간 점 사용
        idx_1 = len(segment_points) // 3
        idx_2 = 2 * len(segment_points) // 3
        P1 = segment_points[idx_1]
        P2 = segment_points[idx_2]
    else:
        # 적으면 선형 보간
        direction = P3 - P0
        P1 = P0 + direction / 3
        P2 = P0 + 2 * direction / 3
    
    control_points = np.array([P0, P1, P2, P3])
    
    # 4. ✅ Bézier curve 생성
    num_bezier_points = max(20, int(cumulative_dist / 0.05))  # 5cm 간격
    bezier_points = bezier_curve(control_points, num_bezier_points)
    
    return bezier_points


def generate_bezier_from_waypoints(waypoints, num_points_per_segment=50):
    """
    Waypoint들을 연결하는 연속적인 Bézier curves 생성
    """
    if len(waypoints) < 2:
        return None
    
    all_curves = []
    
    for i in range(len(waypoints) - 1):
        P0 = np.array(waypoints[i])
        P3 = np.array(waypoints[i + 1])
        
        # 중간 제어점 (직선의 1/3, 2/3 지점)
        direction = P3 - P0
        P1 = P0 + direction / 3
        P2 = P0 + 2 * direction / 3
        
        control_points = np.array([P0, P1, P2, P3])
        curve = bezier_curve(control_points, num_points_per_segment)
        
        all_curves.append(curve)
    
    return np.vstack(all_curves)
