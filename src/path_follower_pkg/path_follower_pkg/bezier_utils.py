#!/usr/bin/env python3
import numpy as np
import math


def bezier_curve(control_points, num_points=50):
    """Bézier curve 생성"""
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 2))
    
    for i, t in enumerate(t_values):
        point = np.zeros(2)
        for j in range(n + 1):
            binomial = math.comb(n, j)
            bernstein = binomial * (t ** j) * ((1 - t) ** (n - j))
            point += bernstein * control_points[j]
        curve[i] = point
    
    return curve


def _distance_point_to_segment(pt, a, b):
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < 1e-9:
        return np.linalg.norm(pt - a), a

    t = np.clip(np.dot(pt - a, ab) / denom, 0.0, 1.0)
    proj = a + t * ab
    return np.linalg.norm(pt - proj), proj


def _apply_constraint_offset(control_points, constraints, avoid_margin=0.45, max_offset=0.8):
    if not constraints:
        return control_points

    p0, p1, p2, p3 = control_points
    path_vec = p3 - p0
    path_norm = np.linalg.norm(path_vec)
    if path_norm < 1e-6:
        return control_points

    path_dir = path_vec / path_norm
    normal = np.array([-path_dir[1], path_dir[0]])

    offset_vec = np.zeros(2)

    for cp in constraints:
        # 전방(경로 진행방향 90도 내) 장애물만 반영
        ahead_vec = cp - p0
        proj = np.dot(ahead_vec, path_dir)
        if proj < 0:
            continue

        dist, _ = _distance_point_to_segment(cp, p0, p3)
        if dist >= avoid_margin:
            continue

        side = np.cross(path_dir, cp - p0)
        side_sign = 1.0 if side >= 0 else -1.0

        strength = (avoid_margin - dist) / avoid_margin
        # 가까울수록 강하게, 전방일수록 가중
        forward_weight = np.clip(proj / (path_norm + 1e-6), 0.2, 1.0)
        offset_vec += normal * side_sign * strength * forward_weight

    norm_offset = np.linalg.norm(offset_vec)
    if norm_offset < 1e-6:
        return control_points

    clipped = min(norm_offset, max_offset)
    offset_vec = offset_vec / norm_offset * clipped
    control_points[1] = control_points[1] + offset_vec
    control_points[2] = control_points[2] + offset_vec

    return control_points


def split_global_to_local_bezier(global_path, robot_pos, lookahead_dist=0.5, constraints=None):
    """
    ✅ 개선된 Local Bézier: shortcut 방지 + 제약점 기반 경로 틀기
    """
    if global_path is None or len(global_path.poses) < 2:
        return None
    
    # Global path를 numpy array로 변환
    global_points = np.array([
        [pose.pose.position.x, pose.pose.position.y]
        for pose in global_path.poses
    ])
    
    # 1. Nearest point 찾기
    distances = np.linalg.norm(global_points - robot_pos, axis=1)
    nearest_idx = np.argmin(distances)
    
    # 2. ✅ Lookahead 구간 추출 (0.5m로 단축)
    segment_points = []
    segment_indices = []
    cumulative_dist = 0.0
    
    for i in range(nearest_idx, len(global_points)):
        segment_points.append(global_points[i])
        segment_indices.append(i)
        
        if i > nearest_idx:
            segment_dist = np.linalg.norm(global_points[i] - global_points[i-1])
            cumulative_dist += segment_dist
            
            if cumulative_dist >= lookahead_dist:
                break
    
    if len(segment_points) < 4:
        # 점이 부족하면 그대로 반환 (Bézier 생성 안 함)
        return np.array(segment_points) if len(segment_points) >= 2 else None
    
    segment_points = np.array(segment_points)
    
    # 3. ✅ 제어점 선택 (Global path 위의 실제 점 사용)
    n = len(segment_points)
    
    P0 = segment_points[0]        # 시작점
    P1 = segment_points[n // 4]   # 1/4 지점 (실제 경로 위)
    P2 = segment_points[3 * n // 4]  # 3/4 지점 (실제 경로 위)
    P3 = segment_points[-1]       # 끝점

    control_points = np.array([P0, P1, P2, P3])

    # ✅ 동적 장애물 cp 제약 적용: 제어점 틀기
    control_points = _apply_constraint_offset(control_points, constraints or [])
    
    # 4. ✅ Bézier curve 생성 (더 조밀하게)
    num_bezier_points = max(15, int(cumulative_dist / 0.03))  # 3cm 간격
    bezier_points = bezier_curve(control_points, num_bezier_points)
    
    # 5. ✅ Shortcut 방지: 원본 경로와 너무 멀어지면 원본 반환
    # 각 Bézier 점이 원본 경로로부터 얼마나 떨어졌는지 체크
    max_deviation = 0.0
    for bp in bezier_points:
        min_dist_to_original = np.min(np.linalg.norm(segment_points - bp, axis=1))
        max_deviation = max(max_deviation, min_dist_to_original)
    
    # 제약 기반 회피 시 허용 편차를 넉넉히, 아닐 때는 10cm로 제한
    deviation_limit = 0.25 if constraints else 0.1
    if max_deviation > deviation_limit:
        return segment_points
    
    return bezier_points


def generate_bezier_from_waypoints(waypoints, num_points_per_segment=50):
    """Waypoint들을 연결하는 연속적인 Bézier curves 생성"""
    if len(waypoints) < 2:
        return None
    
    all_curves = []
    
    for i in range(len(waypoints) - 1):
        P0 = np.array(waypoints[i])
        P3 = np.array(waypoints[i + 1])
        
        direction = P3 - P0
        P1 = P0 + direction / 3
        P2 = P0 + 2 * direction / 3
        
        control_points = np.array([P0, P1, P2, P3])
        curve = bezier_curve(control_points, num_points_per_segment)
        
        all_curves.append(curve)
    
    return np.vstack(all_curves)
