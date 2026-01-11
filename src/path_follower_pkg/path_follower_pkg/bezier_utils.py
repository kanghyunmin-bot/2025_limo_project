#!/usr/bin/env python3
import numpy as np
import math


def _lerp2(p, q, t):
    return p + (q - p) * t


def _bernstein_matrix(n: int, t_vals: np.ndarray) -> np.ndarray:
    """Vectorized Bernstein basis matrix for degree-n Bézier evaluation."""

    t = t_vals.reshape(-1, 1)
    i = np.arange(n + 1).reshape(1, -1)
    coeff = np.array([math.comb(n, int(k)) for k in i.ravel()], dtype=float).reshape(1, -1)
    basis = coeff * np.power(t, i) * np.power(1.0 - t, n - i)
    return basis


def _de_casteljau_split(ctrl, t):
    """Split arbitrary-degree Bézier at t, returning (left, right) control points."""

    work = [ctrl]
    n = len(ctrl) - 1
    for _ in range(1, n + 1):
        prev = work[-1]
        cur = np.array([_lerp2(prev[i], prev[i + 1], t) for i in range(len(prev) - 1)])
        work.append(cur)
    left = [work[i][0] for i in range(n + 1)]
    right = [work[n - i][i] for i in range(n + 1)]
    return np.array(left), np.array(right)


def _bezier_eval(ctrl, t):
    ctrl_arr = np.asarray(ctrl, dtype=float)
    n = len(ctrl_arr) - 1
    basis = _bernstein_matrix(n, np.array([t], dtype=float)).reshape(-1)
    return basis @ ctrl_arr


def _bezier_eval_many(ctrl, t_vals: np.ndarray) -> np.ndarray:
    """Evaluate Bézier curve at multiple t using Bernstein matrix (vectorized)."""

    ctrl_arr = np.asarray(ctrl, dtype=float)
    n = len(ctrl_arr) - 1
    basis = _bernstein_matrix(n, np.asarray(t_vals, dtype=float))
    return basis @ ctrl_arr


def _bezier_flatness(ctrl):
    p0, pn = ctrl[0], ctrl[-1]
    dx, dy = pn[0] - p0[0], pn[1] - p0[1]
    denom = math.hypot(dx, dy)
    if denom < 1e-9:
        return 0.0

    mx = 0.0
    for p in ctrl[1:-1]:
        num = abs(dy * (p[0] - p0[0]) - dx * (p[1] - p0[1]))
        mx = max(mx, num / denom)
    return mx


def _aabb_of_points(pts):
    xs = pts[:, 0]
    ys = pts[:, 1]
    return (float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max()))


def _aabb_overlap(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return not (ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0)


def _segment_circle_intersect(a, b, center, r):
    (x1, y1), (x2, y2) = a, b
    (cx, cy) = center
    dx, dy = x2 - x1, y2 - y1
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return math.hypot(cx - x1, cy - y1) <= r + 1e-12

    t = ((cx - x1) * dx + (cy - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    px, py = x1 + t * dx, y1 + t * dy
    return math.hypot(px - cx, py - cy) <= r + 1e-12


def _polygon_contains_point(poly: np.ndarray, point: np.ndarray) -> bool:
    """Check if point is inside convex polygon using cross products (ccw)."""

    x, y = point
    sign = None
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        cur_sign = cross >= 0
        if sign is None:
            sign = cur_sign
        elif sign != cur_sign:
            return False
    return True


def _polygon_circle_overlap(poly: np.ndarray, center: np.ndarray, radius: float) -> bool:
    """Broad-phase hull vs circle check leveraging convex hull containment."""

    aabb_poly = _aabb_of_points(poly)
    if not _aabb_overlap(aabb_poly, (center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius)):
        return False

    # If circle center inside polygon, overlap.
    if _polygon_contains_point(poly, center):
        return True

    # Check distance from circle center to each hull edge.
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        dist, _ = _distance_point_to_segment(center, a, b)
        if dist <= radius:
            return True
    return False


def _find_intervals_bezier_hits(ctrl, obstacles, t0=0.0, t1=1.0, flat_eps=1e-2, max_depth=26):
    """Detect t-intervals where Bézier curve intersects inflated circular obstacles.

    Broad phase: use convex-hull containment to skip most subdivisions.
    Narrow phase: only split when hull overlaps with at least one obstacle.
    """

    ctrl_arr = np.array(ctrl, dtype=float)
    hull = ctrl_arr[:, :2]
    if obstacles:
        overlaps_any = any(
            _polygon_circle_overlap(hull, np.array(c[0], dtype=float), float(c[1]))
            for c in obstacles
        )
        if not overlaps_any:
            return []

    hits = []

    def rec(ctrl_local, a, b, depth):
        hull_local = ctrl_local[:, :2]
        # Broad-phase with convex hull; bail early if no obstacle overlaps the hull.
        if obstacles:
            if not any(
                _polygon_circle_overlap(hull_local, np.array(c[0], dtype=float), float(c[1]))
                for c in obstacles
            ):
                return

        if (depth >= max_depth) or (_bezier_flatness(hull_local) <= flat_eps):
            p0, p1 = hull_local[0], hull_local[-1]
            if any(_segment_circle_intersect(p0, p1, c[0], c[1]) for c in obstacles):
                hits.append((a, b))
            return

        L, R = _de_casteljau_split(ctrl_local, 0.5)
        m = 0.5 * (a + b)
        rec(L, a, m, depth + 1)
        rec(R, m, b, depth + 1)

    rec(ctrl_arr, t0, t1, 0)
    hits.sort()

    merged = []
    for seg in hits:
        if not merged or seg[0] > merged[-1][1] + 1e-6:
            merged.append(list(seg))
        else:
            merged[-1][1] = max(merged[-1][1], seg[1])
    return [(float(a), float(b)) for (a, b) in merged]


def _push_away_from_obstacles(
    control_points,
    obstacles,
    flat_eps=1e-2,
    base_step=0.02,
    clearance=0.0,
    gain=0.5,
    max_passes=4,
):
    """충돌 구간이 사라질 때까지 한쪽 제어점만 밀어내는 간단한 보정.

    - P0/P3는 고정, P1/P2 중 겹치는 구간 쪽만 이동
    - 반경 파라미터를 쓰지 않고, 겹친 정도만큼만 외측으로 민다
    """

    if not obstacles or len(control_points) < 4:
        return control_points

    cp = control_points.copy()
    p0, p1, p2, p3 = cp

    for _ in range(max_passes):
        intervals = _find_intervals_bezier_hits(cp, obstacles, flat_eps=flat_eps)
        if not intervals:
            break

        moved = False
        for (a, b) in intervals:
            # 충돌 구간의 양 끝과 중간 세 점에서만 샘플 → 최소 계산
            for tmid in (a, 0.5 * (a + b), b):
                mid = _bezier_eval(cp, tmid)

                best_obs = None
                best_penetr = None
                for center, radius in obstacles:
                    dist = math.hypot(mid[0] - center[0], mid[1] - center[1])
                    penetr = radius - dist
                    if best_penetr is None or penetr > best_penetr:
                        best_penetr = penetr
                        best_obs = (center, radius, dist)

                if best_obs is None or best_penetr is None:
                    continue

                center, radius, dist = best_obs
                vx, vy = mid[0] - center[0], mid[1] - center[1]
                norm = math.hypot(vx, vy)
                if norm < 1e-6:
                    vx, vy = 1.0, 0.0
                    norm = 1.0
                nx, ny = vx / norm, vy / norm

                # 겹친 만큼만 외측으로, 해당 구간의 제어점 한쪽만 민다
                push = max(base_step, gain * max(0.0, radius - dist + clearance))
                if tmid < 0.5:
                    p1 = (p1[0] + push * nx, p1[1] + push * ny)
                else:
                    p2 = (p2[0] + push * nx, p2[1] + push * ny)
                moved = True

        cp = np.array([p0, p1, p2, p3], dtype=float)
        if not moved:
            base_step *= 1.1

    return cp


def _adaptive_bezier_sample(control_points, max_seg_len=0.14, min_points: int = 8):
    """길이 기반 적응 샘플링 (행렬 평가 사용, 재귀 제거)."""

    ctrl = np.asarray(control_points, dtype=float)
    if len(ctrl) < 2:
        return ctrl

    # 1) 거칠게 샘플해 길이 추정
    coarse_t = np.linspace(0.0, 1.0, num=12)
    coarse_pts = _bezier_eval_many(ctrl, coarse_t)
    seg_len = np.linalg.norm(np.diff(coarse_pts, axis=0), axis=1)
    total_len = float(seg_len.sum())

    n_samples = max(min_points, int(max(total_len / max_seg_len, 1)) + 1)

    # 2) 호 길이 기반 파라미터 재분배
    if n_samples <= len(coarse_t):
        t_vals = np.linspace(0.0, 1.0, num=n_samples)
    else:
        cum_len = np.concatenate([[0.0], np.cumsum(seg_len)])
        cum_len /= max(cum_len[-1], 1e-6)
        target = np.linspace(0.0, 1.0, num=n_samples)
        t_vals = np.interp(target, cum_len, coarse_t)

    return _bezier_eval_many(ctrl, t_vals)


def bezier_curve(control_points, num_points=50):
    """Bézier curve 생성"""
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_points)
    basis = _bernstein_matrix(n, t_values)
    ctrl = np.asarray(control_points, dtype=float)
    return basis @ ctrl


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
    max_offset=1.0,
    clearance=0.08,
    exclusion_radius=0.33,
    max_iterations=4,
    ph_offset_gain=0.35,
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
                1.6,
            )
            along = np.clip(proj / (path_norm + 1e-6), 0.0, 1.0)

            # 곡선을 밀어내는 기본 방향(offset_dir) 외에, 장애물 쪽에 따라
            # 경로 진행방향으로 살짝 미는 "ph offset"을 더해 우회폭을 키운다.
            side = math.copysign(1.0, np.dot(normal, cp - nearest_pt) or 1.0)
            ph_push = path_dir * side * ph_offset_gain * exclusion_push

            # cp4(P3)에 닿기 전까지 지속적으로 밀어내기 위해 최소 힘을 유지
            base_p1 = max(0.5, 1.0 - along)
            base_p2 = max(0.5, 0.55 + along)
            offset_p1 += offset_dir * exclusion_push * base_p1 + ph_push
            offset_p2 += offset_dir * exclusion_push * base_p2 + ph_push

        accum_p1 += offset_p1
        accum_p2 += offset_p2

        cp_array[1] = p1_base + _clip_offset(accum_p1)
        cp_array[2] = p2_base + _clip_offset(accum_p2)

    return cp_array


def _constraints_to_obstacles(constraints, radius=0.33):
    """Point 제약을 원형 장애물로 취급해 하나의 처리 루틴으로 통합한다."""

    if not constraints:
        return []
    return [(np.array(cp, dtype=float), float(radius)) for cp in constraints]


def _filter_obstacles_for_segment(obstacles, p0, p3, window, cap=None):
    """선분 주위 window 안에 있는 원형 장애물만 추린다."""

    if not obstacles:
        return []

    filtered = []
    for center, radius in obstacles:
        dist, _ = _distance_point_to_segment(center, p0, p3)
        if dist <= window + radius:
            filtered.append((center, radius))

    filtered.sort(key=lambda item: _distance_point_to_segment(item[0], p0, p3)[0])
    if cap is not None:
        filtered = filtered[:cap]
    return filtered


def _prefilter_obstacles_by_bbox(obstacles, bbox_min, bbox_max, pad: float = 1.0):
    """간단한 박스 필터로 멀리 있는 장애물을 한번에 제거해 반복 연산 감소."""

    if not obstacles:
        return []

    x0, y0 = bbox_min
    x1, y1 = bbox_max

    filt = []
    for center, radius in obstacles:
        cx, cy = center
        if (cx + radius) < (x0 - pad) or (cx - radius) > (x1 + pad):
            continue
        if (cy + radius) < (y0 - pad) or (cy - radius) > (y1 + pad):
            continue
        filt.append((center, radius))
    return filt


def split_global_to_local_bezier(global_path, robot_pos, lookahead_dist=0.5, constraints=None, constraint_window: float = 0.33):
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

    # ✅ 동적 장애물 cp를 원형으로 가정해 필요할 때만 밀어낸다.
    radius = max(constraint_window, 0.05)
    constraint_obs = _constraints_to_obstacles(constraints or [], radius=radius)
    if constraint_obs:
        seg_window = max(radius * 2.0, 0.9)
        seg_obs = _filter_obstacles_for_segment(
            constraint_obs, P0, P3, window=seg_window, cap=18
        )
        if seg_obs:
            clearance = max(0.08, radius * 0.25)
            control_points = _push_away_from_obstacles(
                control_points,
                seg_obs,
                flat_eps=8e-3,
                base_step=0.022,
                clearance=clearance,
                gain=0.65,
                max_passes=14,
            )

    # 4. ✅ Bézier curve 생성 (길이 기반 분할, 과도한 샘플링 제거)
    bezier_points = _adaptive_bezier_sample(control_points, max_seg_len=0.16, min_points=10)
    
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
    obstacles=None,
    obstacle_window: float = 1.0,
    obstacle_cap: int = 20,
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
    constraint_obs = _constraints_to_obstacles(
        constraint_arr, radius=max(constraint_window, 0.33)
    )
    obstacle_arr = []
    for obs in obstacles or []:
        center, radius = obs
        obstacle_arr.append((np.array(center, dtype=float), float(radius)))

    if len(wp) >= 2:
        bbox_min = np.min(wp, axis=0)
        bbox_max = np.max(wp, axis=0)
        pad = max(constraint_window, obstacle_window, 0.6)
        constraint_obs = _prefilter_obstacles_by_bbox(constraint_obs, bbox_min, bbox_max, pad=pad)
        obstacle_arr = _prefilter_obstacles_by_bbox(obstacle_arr, bbox_min, bbox_max, pad=pad)

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

        control_points = np.array([P0, P1, P2, P3], dtype=float)

        seg_obstacles = []
        if constraint_obs:
            seg_obstacles.extend(
                _filter_obstacles_for_segment(
                    constraint_obs, P0, P3, window=constraint_window, cap=obstacle_cap
                )
            )
        if obstacle_arr:
            seg_obstacles.extend(
                _filter_obstacles_for_segment(
                    obstacle_arr, P0, P3, window=obstacle_window, cap=obstacle_cap
                )
            )

        if seg_obstacles:
            safe_cp = _push_away_from_obstacles(
                control_points,
                seg_obstacles,
                flat_eps=8e-3,
                base_step=0.022,
                clearance=0.08,
                gain=0.65,
                max_passes=5,
            )
            control_points[1], control_points[2] = safe_cp[1], safe_cp[2]

        if ds is not None and ds > 0:
            num_points = max(int(seg_len / ds), 2)
        else:
            num_points = num_points_per_segment

        curve = bezier_curve(control_points, num_points)

        all_curves.append(curve)

    return np.vstack(all_curves) if all_curves else None
