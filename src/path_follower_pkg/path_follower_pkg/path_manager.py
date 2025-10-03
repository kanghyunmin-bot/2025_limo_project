#!/usr/bin/env python3
"""
경로 관리 모듈
- 클릭 포인트 수집
- 경로 생성 및 속도 프로파일 계산
- 경로 메시지 생성
"""
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from path_follower_pkg.spline import UniformSpline


class PathManager:
    """경로 생성 및 관리 클래스"""
    
    def __init__(self, logger, ds=0.03, v_max=0.8, a_long=0.8, a_lat_max=1.5):
        """
        Args:
            logger: ROS logger
            ds: 오버샘플 간격 (m)
            v_max: 최대 선속도 (m/s)
            a_long: 종방향 가속도 (m/s²)
            a_lat_max: 측방향 가속도 (m/s²)
        """
        self.logger = logger
        self.ds = ds
        self.v_max = v_max
        self.a_long = a_long
        self.a_lat_max = a_lat_max
        
        self.click_pts = []
        self.path_xy = None
        self.path_kappa = None
        self.path_v = None
        self.path_len = 0.0
        self.have_path = False

    def add_point(self, x: float, y: float, min_dist=1e-3):
        """새로운 점 추가"""
        p = np.array([x, y], dtype=float)
        if len(self.click_pts) == 0 or np.linalg.norm(p - self.click_pts[-1]) > min_dist:
            self.click_pts.append(p)
            return True
        return False

    def clear(self):
        """경로 초기화"""
        self.click_pts = []
        self.path_xy = None
        self.path_kappa = None
        self.path_v = None
        self.path_len = 0.0
        self.have_path = False
        self.logger.info("Path cleared.")

    def rebuild_path(self):
        """클릭 포인트로부터 경로 재구성"""
        if len(self.click_pts) < 2:
            self.have_path = False
            return False

        xy = np.vstack(self.click_pts)
        keep = [0]
        for i in range(1, len(xy)):
            if np.linalg.norm(xy[i] - xy[keep[-1]]) > 1e-6:
                keep.append(i)
        xy = xy[keep]

        if len(xy) < 2:
            self.have_path = False
            return False

        spline = UniformSpline(xy)
        xy_u, t_u, L = spline.resample_by_arclen(self.ds)
        kappa_u = spline.curvature_at_t(t_u)
        v = self._compute_velocity_profile(kappa_u)

        self.path_xy = xy_u
        self.path_kappa = kappa_u
        self.path_v = v
        self.path_len = (len(xy_u) - 1) * self.ds
        self.have_path = True

        self.logger.info(f"Path updated: {len(xy_u)} points, length≈{self.path_len:.2f} m")
        return True

    def _compute_velocity_profile(self, kappa):
        """곡률 기반 속도 프로파일 계산"""
        v_kappa = np.sqrt(np.clip(self.a_lat_max / (np.abs(kappa) + 1e-6), 0.0, self.v_max**2))
        v_ref = np.minimum(self.v_max, v_kappa)
        v = v_ref.copy()
        
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(max(0.0, v[i-1]**2 + 2*self.a_long*self.ds)))
        
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(max(0.0, v[i+1]**2 + 2*self.a_long*self.ds)))
        
        return v

    def get_path_msg(self, frame_id='odom', stamp=None):
        """nav_msgs/Path 메시지 생성"""
        path = Path()
        path.header.frame_id = frame_id
        if stamp is not None:
            path.header.stamp = stamp
        
        if self.path_xy is not None:
            for (x, y) in self.path_xy:
                ps = PoseStamped()
                ps.header.frame_id = frame_id
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                path.poses.append(ps)
        
        return path

    def get_closest_index(self, x: float, y: float):
        """현재 위치에서 가장 가까운 경로 인덱스 찾기"""
        if self.path_xy is None:
            return None
        
        P = np.array([x, y])
        d = np.linalg.norm(self.path_xy - P, axis=1)
        return int(np.argmin(d))
