#!/usr/bin/env python3
import math
import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


def _segment_hits_circle(a: np.ndarray, b: np.ndarray, center: np.ndarray, radius: float) -> bool:
    (x1, y1), (x2, y2) = a, b
    (cx, cy) = center
    dx, dy = x2 - x1, y2 - y1
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return math.hypot(cx - x1, cy - y1) <= radius + 1e-9

    t = ((cx - x1) * dx + (cy - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    px, py = x1 + t * dx, y1 + t * dy
    return math.hypot(px - cx, py - cy) <= radius + 1e-9


@dataclass
class _Node:
    pos: np.ndarray
    parent: Optional[int]


class RRTPlanner:
    """가벼운 2D RRT 구현.

    - 코스트맵을 원형 장애물 집합으로 보고 빠르게 충돌만 검사
    - goal bias로 수렴 속도를 높이고, 마지막엔 직선 단축(short-cut)으로 포인트 수 최소화
    """

    def __init__(
        self,
        step: float = 0.45,
        max_iter: int = 900,
        goal_sample_rate: float = 0.18,
        min_clearance: float = 0.25,
        shortcut_trials: int = 24,
        seed: Optional[int] = None,
    ):
        self.step = step
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.min_clearance = min_clearance
        self.shortcut_trials = shortcut_trials
        self.rng = random.Random(seed)

    def _collision_free(self, a: np.ndarray, b: np.ndarray, obstacles: Sequence[Tuple[np.ndarray, float]]) -> bool:
        if not obstacles:
            return True
        for center, radius in obstacles:
            if _segment_hits_circle(a, b, center, radius + self.min_clearance):
                return False
        return True

    def _sample(self, bounds: Tuple[np.ndarray, np.ndarray], goal: np.ndarray) -> np.ndarray:
        if self.rng.random() < self.goal_sample_rate:
            return goal
        low, high = bounds
        return np.array([self.rng.uniform(low[i], high[i]) for i in range(2)], dtype=float)

    def _nearest(self, nodes: List[_Node], pt: np.ndarray) -> int:
        dists = [np.linalg.norm(n.pos - pt) for n in nodes]
        return int(np.argmin(dists))

    def _steer(self, src: np.ndarray, dst: np.ndarray) -> np.ndarray:
        vec = dst - src
        dist = np.linalg.norm(vec)
        if dist < 1e-9:
            return src
        scale = min(self.step, dist) / dist
        return src + vec * scale

    def _reconstruct(self, nodes: List[_Node], goal_idx: int) -> List[np.ndarray]:
        path: List[np.ndarray] = []
        idx = goal_idx
        while idx is not None:
            node = nodes[idx]
            path.append(node.pos)
            idx = node.parent
        path.reverse()
        return path

    def _shortcut(self, path: List[np.ndarray], obstacles: Sequence[Tuple[np.ndarray, float]]) -> List[np.ndarray]:
        if len(path) <= 2 or not obstacles:
            return path

        pts = path.copy()
        for _ in range(self.shortcut_trials):
            if len(pts) <= 2:
                break
            i = self.rng.randint(0, len(pts) - 2)
            j = self.rng.randint(i + 1, len(pts) - 1)
            if j - i <= 1:
                continue
            if self._collision_free(pts[i], pts[j], obstacles):
                pts = pts[: i + 1] + pts[j:]
        return pts

    def plan(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        obstacles: Sequence[Tuple[np.ndarray, float]],
        bounds: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    ) -> Optional[List[np.ndarray]]:
        start_arr = np.array(start, dtype=float)
        goal_arr = np.array(goal, dtype=float)

        if bounds is None:
            low = np.minimum(start_arr, goal_arr) - 1.2
            high = np.maximum(start_arr, goal_arr) + 1.2
            bounds = (low, high)

        nodes: List[_Node] = [_Node(start_arr, parent=None)]
        best_idx = 0
        best_dist = np.linalg.norm(goal_arr - start_arr)

        for _ in range(self.max_iter):
            sample = self._sample(bounds, goal_arr)
            nearest_idx = self._nearest(nodes, sample)
            new_pos = self._steer(nodes[nearest_idx].pos, sample)

            if not self._collision_free(nodes[nearest_idx].pos, new_pos, obstacles):
                continue

            nodes.append(_Node(new_pos, parent=nearest_idx))
            dist_to_goal = np.linalg.norm(new_pos - goal_arr)
            if dist_to_goal < best_dist:
                best_dist = dist_to_goal
                best_idx = len(nodes) - 1

            if dist_to_goal <= self.step and self._collision_free(new_pos, goal_arr, obstacles):
                nodes.append(_Node(goal_arr, parent=len(nodes) - 1))
                return self._shortcut(self._reconstruct(nodes, len(nodes) - 1), obstacles)

        if best_idx == 0:
            return None
        return self._shortcut(self._reconstruct(nodes, best_idx), obstacles)

