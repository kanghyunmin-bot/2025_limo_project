#!/usr/bin/env python3
import math
from typing import List, Sequence, Tuple

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
    ):
        self.min_range = min_range
        self.max_range = max_range
        self.min_z = min_z
        self.max_z = max_z
        self.lateral_fov = lateral_fov
        self.angle_resolution = angle_resolution
        self.max_constraints = max_constraints
        self.cluster_distance = cluster_distance

    def _filter_pointcloud(self, cloud: PointCloud2) -> List[Tuple[float, float, float, float]]:
        pts: List[Tuple[float, float, float, float]] = []
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

    def _filter_scan(self, scan: LaserScan) -> List[Tuple[float, float, float, float]]:
        pts: List[Tuple[float, float, float, float]] = []
        half_fov = self.lateral_fov * 0.5

        angle = scan.angle_min
        for rng in scan.ranges:
            if not math.isfinite(rng):
                angle += scan.angle_increment
                continue

            if rng < self.min_range or rng > self.max_range:
                angle += scan.angle_increment
                continue

            if abs(angle) > half_fov:
                angle += scan.angle_increment
                continue

            x = rng * math.cos(angle)
            y = rng * math.sin(angle)
            pts.append((rng, angle, x, y))
            angle += scan.angle_increment

        return pts

    def _cluster_by_density(self, polar_points: Sequence[Tuple[float, float, float, float]]):
        if not polar_points:
            return []

        clusters: List[List[Tuple[float, float, float, float]]] = []
        current: List[Tuple[float, float, float, float]] = []
        prev = None

        ordered = sorted(polar_points, key=lambda p: p[1])

        for pt in ordered:
            if prev is None:
                current = [pt]
            else:
                _, _, prev_x, prev_y = prev
                _, _, cur_x, cur_y = pt
                if math.hypot(cur_x - prev_x, cur_y - prev_y) <= self.cluster_distance:
                    current.append(pt)
                else:
                    clusters.append(current)
                    current = [pt]
            prev = pt

        if current:
            clusters.append(current)

        centroids: List[Tuple[float, float, float, float]] = []
        for cluster in clusters:
            xs = [p[2] for p in cluster]
            ys = [p[3] for p in cluster]
            cx = float(np.mean(xs))
            cy = float(np.mean(ys))
            rng = math.hypot(cx, cy)
            angle = math.atan2(cy, cx)
            centroids.append((rng, angle, cx, cy))

        return centroids

    def _bin_and_select(self, candidates: Sequence[Tuple[float, float, float, float]]):
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

    def build_constraints(self, data) -> List[np.ndarray]:
        """Polar binning으로 최근접 장애물을 cp로 선택.

        LaserScan은 인접 포인트 뭉침(밀도)을 클러스터로 묶어 대표점을 cp로 삼고,
        PointCloud2는 기존과 동일하게 거리/높이/FOV 필터 후 최근접점을 선택한다.
        """

        if isinstance(data, LaserScan):
            polar_pts = self._filter_scan(data)
            clustered = self._cluster_by_density(polar_pts)
            return self._bin_and_select(clustered)

        if isinstance(data, PointCloud2):
            candidates = self._filter_pointcloud(data)
            return self._bin_and_select(candidates)

        return []
