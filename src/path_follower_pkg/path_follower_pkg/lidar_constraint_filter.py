#!/usr/bin/env python3
import math
from typing import List

import numpy as np
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2


class LidarConstraintFilter:
    """PointCloud2 -> 제약(cp) 포인트 추출기."""

    def __init__(
        self,
        min_range: float = 0.2,
        max_range: float = 5.0,
        min_z: float = -0.2,
        max_z: float = 0.8,
        lateral_fov: float = math.radians(140.0),
        angle_resolution: float = math.radians(10.0),
        max_constraints: int = 20,
    ):
        self.min_range = min_range
        self.max_range = max_range
        self.min_z = min_z
        self.max_z = max_z
        self.lateral_fov = lateral_fov
        self.angle_resolution = angle_resolution
        self.max_constraints = max_constraints

    def _filter_points(self, cloud: PointCloud2) -> List[np.ndarray]:
        pts = []
        half_fov = self.lateral_fov * 0.5

        for x, y, z in point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        ):
            if z < self.min_z or z > self.max_z:
                continue

            rng = math.hypot(x, y)
            if rng < self.min_range or rng > self.max_range:
                continue

            angle = math.atan2(y, x)
            if abs(angle) > half_fov:
                continue

            pts.append((rng, angle, x, y))

        return pts

    def build_constraints(self, cloud: PointCloud2) -> List[np.ndarray]:
        """Polar binning으로 최근접 장애물을 cp로 선택."""
        candidates = self._filter_points(cloud)
        if not candidates:
            return []

        bins = {}
        half_fov = self.lateral_fov * 0.5

        for rng, angle, x, y in candidates:
            bin_idx = int((angle + half_fov) // self.angle_resolution)
            best = bins.get(bin_idx)
            if best is None or rng < best[0]:
                bins[bin_idx] = (rng, x, y)

        selected = sorted(bins.values(), key=lambda t: t[0])[: self.max_constraints]
        return [np.array([x, y]) for _, x, y in selected]
