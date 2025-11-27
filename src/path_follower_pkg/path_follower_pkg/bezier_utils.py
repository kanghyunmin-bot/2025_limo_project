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


def _apply_constraint_offset(
    control_points,
    constraints,
    avoid_margin=0.45,
    max_offset=1.2,
    clearance=0.12,
    exclusion_radius=0.8,
    max_iterations=4,
    ph_offset_gain=0.4,
):
    """
    제약(cp)이 Bézier 구간을 덮을 때만 중간 제어점(P1, P2)만 경로 밖으로 밀어낸다.

    - P0(로봇 위치)와 P3(글로벌 경로 복귀점)는 항상 고정
    - P1은 시작 쪽 회피, P2는 끝점 쪽 회피를 주로 담당
    """

    if not constraints:
        return control_points

    # 원본 제어점은 그대로 두고 중간 제어점만 이동시키기 위해 사본 사용
    cp_array = control_points.copy()
    p0, p1, p2, p3 = cp_array

    path_vec = p3 - p0
    path_norm = np.linalg.norm(path_vec)
    if path_norm < 1e-6:
        return control_points

    path_dir = path_vec / path_norm
    normal = np.array([-path_dir[1], path_dir[0]])

    # 중간 제어점 누적 오프셋 (P0, P3 고정)
    p1_base = p1.copy()
    p2_base = p2.copy()
    accum_p1 = np.zeros(2)
    accum_p2 = np.zeros(2)

    def _clip_offset(vec):
        norm = np.linalg.norm(vec)
        if norm < 1e-6:
            return np.zeros_like(vec)
        return vec / norm * min(norm, max_offset)

    def _closest_to_curve(cp, curve):
        distances = np.linalg.norm(curve - cp, axis=1)
        idx = int(np.argmin(distances))
        return float(distances[idx]), curve[idx]

    # 제약 영역(원형 exclusion)을 만족할 때까지 반복적으로 밀어내기
    for _ in range(max_iterations):
        base_curve = bezier_curve(cp_array, num_points=80)
        overlapping = []

        for cp in constraints:
            curve_dist, nearest_pt = _closest_to_curve(cp, base_curve)
            if curve_dist < exclusion_radius:
                overlapping.append((cp, curve_dist, nearest_pt))

        if not overlapping:
            break

        offset_p1 = np.zeros(2)
        offset_p2 = np.zeros(2)

        for cp, curve_dist, nearest_pt in overlapping:
            ahead_vec = cp - p0
            proj = np.dot(ahead_vec, path_dir)
            if proj < -0.1:  # 뒤쪽 장애물은 무시
                continue

            curve_to_cp = cp - nearest_pt
            dist_norm = np.linalg.norm(curve_to_cp)
            if dist_norm < 1e-6:
                curve_to_cp = normal
                dist_norm = np.linalg.norm(curve_to_cp)

            # curve_to_cp는 경로→장애물 방향이므로, 해당 방향으로 제어점을 밀어내면
            # 겹침을 풀 수 있다.
            offset_dir = curve_to_cp / dist_norm
            exclusion_push = np.clip(
                (exclusion_radius - curve_dist + clearance)
                / max(exclusion_radius, 1e-6),
                0.0,
                2.5,
            )
            along = np.clip(proj / (path_norm + 1e-6), 0.0, 1.0)

            # 곡선을 밀어내는 기본 방향(offset_dir) 외에, 장애물 쪽에 따라
            # 경로 진행방향으로 살짝 미는 "ph offset"을 더해 우회폭을 키운다.
            side = math.copysign(1.0, np.dot(normal, cp - nearest_pt) or 1.0)
            ph_push = path_dir * side * ph_offset_gain * exclusion_push

            # cp4(P3)에 닿기 전까지 지속적으로 밀어내기 위해 최소 힘을 유지
            base_p1 = max(0.6, 1.1 - along)
            base_p2 = max(0.6, 0.6 + along)
            offset_p1 += offset_dir * exclusion_push * base_p1 + ph_push
            offset_p2 += offset_dir * exclusion_push * base_p2 + ph_push

        accum_p1 += offset_p1
        accum_p2 += offset_p2

        cp_array[1] = p1_base + _clip_offset(accum_p1)
        cp_array[2] = p2_base + _clip_offset(accum_p2)

    return cp_array


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


def generate_bezier_from_waypoints(
    waypoints,
    num_points_per_segment=50,
    ds=None,
    constraints=None,
    constraint_window: float = 0.9,
    tension: float = 0.28,
):
    """Waypoint 기반 글로벌 Bézier 체인 생성.

    - 인접 방향을 이용해 직각 경로를 자동으로 둥글게 만들고,
      Nav2 코스트맵/제약점이 근접한 세그먼트는 중간 제어점만 밀어낸다.
    - ds를 주면 구간 길이에 비례해 점을 촘촘히 찍어 spline과 유사한 밀도를 유지.
    """
    if len(waypoints) < 2:
        return None

    wp = np.array(waypoints, dtype=float)
    n = len(wp)
    tangents = []
    for i in range(n):
        if i == 0:
            tvec = wp[i + 1] - wp[i]
        elif i == n - 1:
            tvec = wp[i] - wp[i - 1]
        else:
            tvec = wp[i + 1] - wp[i - 1]
        norm = np.linalg.norm(tvec)
        tangents.append(tvec / norm if norm > 1e-6 else np.zeros(2))
    tangents = np.array(tangents)

    all_curves = []
    constraint_arr = [np.array(c) for c in (constraints or [])]

    for i in range(n - 1):
        P0 = wp[i]
        P3 = wp[i + 1]

        direction = P3 - P0
        seg_len = float(np.linalg.norm(direction))
        if seg_len < 1e-6:
            continue

        # 코너를 둥글게 만들도록 양끝 접선 방향을 활용
        t0 = tangents[i]
        t1 = tangents[i + 1]
        P1 = P0 + t0 * tension * seg_len
        P2 = P3 - t1 * tension * seg_len

        # 세그먼트 주변 코스트맵 제약을 중간 제어점에만 반영
        if constraint_arr:
            seg_constraints = []
            for cp in constraint_arr:
                dist, _ = _distance_point_to_segment(cp, P0, P3)
                if dist <= constraint_window:
                    seg_constraints.append(cp)
            if seg_constraints:
                cp_adjusted = _apply_constraint_offset(
                    np.array([P0, P1, P2, P3], dtype=float),
                    seg_constraints,
                    exclusion_radius=max(constraint_window, 0.6),
                )
                P1, P2 = cp_adjusted[1], cp_adjusted[2]

        if ds is not None and ds > 0:
            num_points = max(int(seg_len / ds), 2)
        else:
            num_points = num_points_per_segment

        control_points = np.array([P0, P1, P2, P3])
        curve = bezier_curve(control_points, num_points)

        all_curves.append(curve)

    return np.vstack(all_curves) if all_curves else None
