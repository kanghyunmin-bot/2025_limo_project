#!/usr/bin/env python3
import math
from typing import List

import numpy as np
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2


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
        cluster_distance: float = 0.3,
        inflate_clearance: float = 0.35,
    ):
        self.min_range = min_range
        self.max_range = max_range
        self.min_z = min_z
        self.max_z = max_z
        self.lateral_fov = lateral_fov
        self.angle_resolution = angle_resolution
        self.max_constraints = max_constraints
        self.cluster_distance = cluster_distance
        self.inflate_clearance = inflate_clearance

    def _filter_pointcloud(self, cloud: PointCloud2) -> List[np.ndarray]:
        pts: List[np.ndarray] = []
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

            pts.append(np.array([x, y], dtype=float))

        if not pts:
            return []

        arr = np.vstack(pts)
        norms = np.linalg.norm(arr, axis=1)
        scale = np.where(norms > 1e-6, (norms + self.inflate_clearance) / norms, 0.0)
        inflated = arr * scale[:, None]
        step = max(1, int(self.angle_resolution / max(abs(self.angle_resolution), 1e-6)))
        return [pt for pt in inflated[::step][: self.max_constraints]]

    def _process_scan_numpy(self, scan: LaserScan) -> List[np.ndarray]:
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = np.linspace(
            scan.angle_min,
            scan.angle_min + scan.angle_increment * (len(ranges) - 1),
            num=len(ranges),
            dtype=float,
        )

        half_fov = 0.5 * self.lateral_fov
        mask = (
            np.isfinite(ranges)
            & (ranges > self.min_range)
            & (ranges < self.max_range)
            & (np.abs(angles) < half_fov)
        )
        if not np.any(mask):
            return []

        r_sel = ranges[mask]
        a_sel = angles[mask]
        xs = r_sel * np.cos(a_sel)
        ys = r_sel * np.sin(a_sel)

        pts = np.stack([xs, ys], axis=1)
        step = max(1, int(math.ceil(self.angle_resolution / max(abs(scan.angle_increment), 1e-6))))
        pts = pts[::step]

        norms = np.linalg.norm(pts, axis=1)
        scale = np.where(norms > 1e-6, (norms - self.inflate_clearance) / norms, 0.0)
        scale = np.clip(scale, 0.0, None)
        inflated = pts * scale[:, None]

        if self.max_constraints and inflated.shape[0] > self.max_constraints:
            inflated = inflated[: self.max_constraints]

        return [pt for pt in inflated]

    def build_constraints(self, data) -> List[np.ndarray]:
        """Polar binning으로 최근접 장애물을 cp로 선택.

        LaserScan은 인접 포인트 뭉침(밀도)을 클러스터로 묶어 대표점을 cp로 삼고,
        PointCloud2는 기존과 동일하게 거리/높이/FOV 필터 후 최근접점을 선택한다.
        """

        if isinstance(data, LaserScan):
            return self._process_scan_numpy(data)

        if isinstance(data, PointCloud2):
            return self._filter_pointcloud(data)

        return []
