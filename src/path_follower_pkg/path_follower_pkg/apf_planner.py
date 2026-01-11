#!/usr/bin/env python3
"""Lightweight 2D Artificial Potential Field (APF) planner.

The planner pulls the path toward the goal while repelling from circular
obstacles. It returns a polyline that can be further smoothed by the
Bézier/spline stages in :mod:`path_manager`.

Focused on low overhead: all math uses NumPy arrays and a fixed iteration
budget so it can run inline when the user switches planner mode via the GUI.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class APFParams:
    step: float = 0.25
    max_iter: int = 220
    attract_gain: float = 1.0
    repel_gain: float = 0.9
    influence_dist: float = 1.2
    goal_tolerance: float = 0.18
    stall_tolerance: float = 0.04
    stall_window: int = 18


class APFPlanner:
    """Simple APF implementation for short segments.

    Obstacles are circles ``(center, radius)``. The planner stops when it is
    within ``goal_tolerance`` of the goal or the step budget is exhausted.
    """

    def __init__(self, params: Optional[APFParams] = None):
        self.params = params or APFParams()

    def _repulsive_force(self, pos: np.ndarray, obstacles: Sequence[Tuple[np.ndarray, float]]) -> np.ndarray:
        if not obstacles:
            return np.zeros(2, dtype=float)

        force = np.zeros(2, dtype=float)
        for center, radius in obstacles:
            offset = pos - center
            dist = np.linalg.norm(offset) + 1e-12
            if dist > self.params.influence_dist + radius:
                continue

            # Classic APF repulsion scaled by squared distance to reduce jitter
            overlap = max(self.params.influence_dist + radius - dist, 0.0)
            scale = self.params.repel_gain * (overlap / (dist * dist))
            force += (offset / dist) * scale
        return force

    def plan(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        obstacles: Sequence[Tuple[np.ndarray, float]],
    ) -> Optional[List[np.ndarray]]:
        start_arr = np.array(start, dtype=float)
        goal_arr = np.array(goal, dtype=float)

        if np.linalg.norm(goal_arr - start_arr) < 1e-6:
            return [start_arr]

        pts: List[np.ndarray] = [start_arr]
        recent_progress: List[float] = []

        for _ in range(self.params.max_iter):
            pos = pts[-1]
            to_goal = goal_arr - pos
            dist_goal = np.linalg.norm(to_goal)

            if dist_goal <= self.params.goal_tolerance:
                pts.append(goal_arr)
                return pts

            attract = (to_goal / (dist_goal + 1e-12)) * self.params.attract_gain
            repel = self._repulsive_force(pos, obstacles)
            direction = attract + repel

            norm = np.linalg.norm(direction)
            if norm < 1e-9:
                # No clear direction; stop to avoid oscillation
                break

            step = min(self.params.step, dist_goal)
            new_pos = pos + direction / norm * step
            pts.append(new_pos)

            # Track recent progress to detect stalls
            delta = np.linalg.norm(new_pos - pos)
            recent_progress.append(delta)
            if len(recent_progress) > self.params.stall_window:
                recent_progress.pop(0)
                avg_move = sum(recent_progress) / len(recent_progress)
                if avg_move < self.params.stall_tolerance:
                    break

        return None

