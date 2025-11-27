#!/usr/bin/env python3
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid


class CostmapConstraintFilter:
    """Build constraint points from a Nav2-style global costmap.

    The filter keeps obstacle cells above a cost threshold, limits them to a
    window around the robot, and only returns points that sit near the current
    global path segment so Bézier bending reacts where it matters.
    """

    def __init__(
        self,
        cost_threshold: int = 50,
        search_radius: float = 3.0,
        path_window: float = 0.8,
        inflate_margin: float = 0.35,
        robot_radius: float = 0.20,
        safety_margin: float = 0.05,
        stride: int = 2,
        max_constraints: int = 60,
    ):
        self.cost_threshold = cost_threshold
        self.search_radius = search_radius
        self.path_window = path_window
        self.inflate_margin = inflate_margin
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.stride = max(1, stride)
        self.max_constraints = max_constraints

        self._grid: Optional[np.ndarray] = None
        self._resolution: float = 0.0
        self._origin = np.zeros(2)
        self._frame_id: Optional[str] = None

    def build_obstacle_circles(self) -> List[Tuple[np.ndarray, float]]:
        """Convert occupied cells into inflated circular obstacles for Bézier collision checks.

        The radius is conservative: half a cell's diagonal plus robot radius and a safety margin.
        """

        if self._grid is None:
            return []

        mask = self._grid >= self.cost_threshold
        if not np.any(mask):
            return []

        ys, xs = np.nonzero(mask)
        centers = self._world_from_index(xs, ys)

        cell_diag = self._resolution * math.sqrt(2) * 0.5
        radius = cell_diag + self.robot_radius + self.safety_margin

        return [(c, float(radius + self.inflate_margin)) for c in centers]

    def update_costmap(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0:
            self._grid = None
            return

        grid = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        self._grid = grid
        self._resolution = msg.info.resolution
        self._origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y], dtype=float)
        self._frame_id = msg.header.frame_id

    def _world_from_index(self, ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
        return self._origin + np.stack([ix + 0.5, iy + 0.5], axis=1) * self._resolution

    def _path_points_near_robot(self, path: Sequence, robot_xy: np.ndarray, radius: float) -> np.ndarray:
        pts: List[np.ndarray] = []
        radius_sq = radius * radius
        for pose in path:
            px = pose.pose.position.x
            py = pose.pose.position.y
            if (px - robot_xy[0]) ** 2 + (py - robot_xy[1]) ** 2 <= radius_sq:
                pts.append(np.array([px, py], dtype=float))
        return np.vstack(pts) if pts else np.empty((0, 2))

    def _inflate_toward_path(self, candidate: np.ndarray, nearest: np.ndarray) -> np.ndarray:
        offset = candidate - nearest
        norm = np.linalg.norm(offset)
        if norm < 1e-6:
            return nearest
        scale = (norm + self.inflate_margin) / norm
        return nearest + offset * scale

    def _path_points(self, path: Sequence, limit_radius: float | None = None, center: np.ndarray | None = None) -> np.ndarray:
        if path is None:
            return np.empty((0, 2))

        pts: List[np.ndarray] = []
        radius_sq = None if limit_radius is None else limit_radius * limit_radius

        for pose in path:
            px = pose.pose.position.x
            py = pose.pose.position.y

            if radius_sq is not None and center is not None:
                if (px - center[0]) ** 2 + (py - center[1]) ** 2 > radius_sq:
                    continue

            pts.append(np.array([px, py], dtype=float))

        return np.vstack(pts) if pts else np.empty((0, 2))

    def build_constraints(self, global_path, robot_xy: np.ndarray, logger=None) -> List[np.ndarray]:
        if self._grid is None or global_path is None or len(global_path.poses) == 0:
            return []

        path_frame = global_path.header.frame_id
        if self._frame_id and path_frame and self._frame_id != path_frame and logger:
            logger.warn(
                f"⚠️ Costmap frame '{self._frame_id}' != path frame '{path_frame}'. Assuming same frame for constraints."
            )

        mask = self._grid >= self.cost_threshold
        mask = mask[:: self.stride, :: self.stride]
        if not np.any(mask):
            return []

        ys, xs = np.nonzero(mask)
        xs = xs * self.stride
        ys = ys * self.stride

        world_pts = self._world_from_index(xs, ys)
        # Robot-proximate filtering
        d_robot = np.linalg.norm(world_pts - robot_xy, axis=1)
        keep_robot = d_robot <= self.search_radius
        if not np.any(keep_robot):
            return []
        world_pts = world_pts[keep_robot]

        path_pts = self._path_points(global_path.poses, limit_radius=self.search_radius + self.path_window, center=robot_xy)
        if path_pts.size == 0:
            return []

        diffs = world_pts[:, None, :] - path_pts[None, :, :]
        dist_sq = np.sum(diffs ** 2, axis=2)
        nearest_idx = np.argmin(dist_sq, axis=1)
        min_dist = np.sqrt(dist_sq[np.arange(dist_sq.shape[0]), nearest_idx])

        keep_path = min_dist <= self.path_window
        if not np.any(keep_path):
            return []

        world_pts = world_pts[keep_path]
        nearest_idx = nearest_idx[keep_path]
        min_dist = min_dist[keep_path]

        constraints: List[np.ndarray] = []
        for pt, idx, d in zip(world_pts, nearest_idx, min_dist):
            nearest = path_pts[int(idx)]
            inflated = self._inflate_toward_path(pt, nearest)
            # If the raw point is already outside the window, skip (extra guard)
            if d > self.path_window:
                continue
            constraints.append(inflated)
            if len(constraints) >= self.max_constraints:
                break

        return constraints

    def build_constraints_for_path(self, global_path, logger=None) -> List[np.ndarray]:
        """Extract constraint points along the whole global path (Nav2-style global planner output).

        This is used for global Bézier fitting so that right-angled grid paths are
        slightly bent away from high-cost cells before local avoidance is applied.
        """

        if self._grid is None or global_path is None or len(global_path.poses) == 0:
            return []

        path_frame = global_path.header.frame_id
        if self._frame_id and path_frame and self._frame_id != path_frame and logger:
            logger.warn(
                f"⚠️ Costmap frame '{self._frame_id}' != path frame '{path_frame}'. Assuming same frame for constraints."
            )

        mask = self._grid >= self.cost_threshold
        mask = mask[:: self.stride, :: self.stride]
        if not np.any(mask):
            return []

        ys, xs = np.nonzero(mask)
        xs = xs * self.stride
        ys = ys * self.stride
        world_pts = self._world_from_index(xs, ys)

        path_pts = self._path_points(global_path.poses)
        if path_pts.size == 0:
            return []

        diffs = world_pts[:, None, :] - path_pts[None, :, :]
        dist_sq = np.sum(diffs ** 2, axis=2)
        nearest_idx = np.argmin(dist_sq, axis=1)
        min_dist = np.sqrt(dist_sq[np.arange(dist_sq.shape[0]), nearest_idx])

        keep_path = min_dist <= self.path_window
        if not np.any(keep_path):
            return []

        world_pts = world_pts[keep_path]
        nearest_idx = nearest_idx[keep_path]
        min_dist = min_dist[keep_path]

        constraints: List[np.ndarray] = []
        for pt, idx, d in zip(world_pts, nearest_idx, min_dist):
            nearest = path_pts[int(idx)]
            inflated = self._inflate_toward_path(pt, nearest)
            if d > self.path_window:
                continue
            constraints.append(inflated)
            if len(constraints) >= self.max_constraints:
                break

        return constraints
