#!/usr/bin/env python3
"""
Spline 모듈 (UniformSpline 기반)
"""
import numpy as np


class UniformSpline:
    """Natural Cubic Spline + 호길이 균일 재샘플 + 곡률 계산"""
    
    def __init__(self, xy: np.ndarray):
        assert xy.shape[0] >= 2
        self.xy_raw = xy.astype(float)
        self._fit()

    def _fit(self):
        d = np.linalg.norm(np.diff(self.xy_raw, axis=0), axis=1)
        s = np.concatenate(([0.0], np.cumsum(d)))
        s_total = s[-1] if s[-1] > 1e-6 else 1.0
        t = s / s_total

        def cubic_spline_coeff(x, t):
            n = len(x)
            h = np.diff(t)
            A = np.zeros((n, n))
            b = np.zeros(n)
            A[0,0] = 1.0
            A[-1,-1] = 1.0
            for i in range(1, n-1):
                A[i, i-1] = h[i-1]/6.0
                A[i, i]   = (h[i-1]+h[i])/3.0
                A[i, i+1] = h[i]/6.0
                b[i] = (x[i+1]-x[i])/h[i] - (x[i]-x[i-1])/h[i-1]
            m = np.linalg.solve(A, b)
            return m

        self.t = t
        self.x = self.xy_raw[:,0]
        self.y = self.xy_raw[:,1]
        self.mx = cubic_spline_coeff(self.x, t)
        self.my = cubic_spline_coeff(self.y, t)

    def _eval_1d(self, q, tq, x, m):
        i = np.searchsorted(q, tq) - 1
        i = np.clip(i, 0, len(q)-2)
        h = q[i+1] - q[i]
        a = (q[i+1] - tq) / h
        b = (tq - q[i])   / h
        s = (a*x[i] + b*x[i+1] +
             ((a**3 - a)*m[i] + (b**3 - b)*m[i+1])*(h**2)/6.0)
        return s

    def eval_xy(self, tq):
        X = self._eval_1d(self.t, tq, self.x, self.mx)
        Y = self._eval_1d(self.t, tq, self.y, self.my)
        return X, Y

    def eval_dxy_dt(self, tq):
        def deriv(q, tq, x, m):
            i = np.searchsorted(q, tq) - 1
            i = np.clip(i, 0, len(q)-2)
            h = q[i+1] - q[i]
            a = (q[i+1] - tq) / h
            b = (tq - q[i])   / h
            ds = ((x[i+1]-x[i])/h +
                  ((-3*a*a+1)*m[i] + (3*b*b-1)*m[i+1])*(h)/6.0)
            return ds
        return deriv(self.t, tq, self.x, self.mx), deriv(self.t, tq, self.y, self.my)

    def eval_d2xy_dt2(self, tq):
        def d2(q, tq, x, m):
            i = np.searchsorted(q, tq) - 1
            i = np.clip(i, 0, len(q)-2)
            h = q[i+1] - q[i]
            a = (q[i+1] - tq) / h
            b = (tq - q[i])   / h
            d2s = ((-6*a)*m[i] + (6*b)*m[i+1])*(1/6.0)
            return d2s
        return d2(self.t, tq, self.x, self.mx), d2(self.t, tq, self.y, self.my)

    def resample_by_arclen(self, ds=0.02):
        tt = np.linspace(0, 1, 2000)
        dx, dy = self.eval_dxy_dt(tt)
        spd = np.hypot(dx, dy)
        s = np.cumsum((spd[:-1] + spd[1:]) * (tt[1]-tt[0]) * 0.5)
        s = np.concatenate(([0.0], s))
        L = s[-1] if s[-1] > 1e-9 else 1.0
        s_target = np.arange(0.0, L+1e-9, ds)
        t_out = np.interp(s_target, s, tt)
        x_out, y_out = self.eval_xy(t_out)
        return np.vstack([x_out, y_out]).T, t_out, L

    def curvature_at_t(self, tq):
        dx, dy   = self.eval_dxy_dt(tq)
        ddx, ddy = self.eval_d2xy_dt2(tq)
        denom = (dx*dx + dy*dy)**1.5 + 1e-9
        return (dx*ddy - dy*ddx) / denom


def generate_smooth_path(waypoints, ds=0.1, min_turn_radius=0.5):
    """스플라인 경로 생성"""
    if len(waypoints) < 2:
        return waypoints
    
    try:
        # UniformSpline으로 경로 생성
        spline = UniformSpline(waypoints)
        path, _, _ = spline.resample_by_arclen(ds=ds)
        return path
    
    except Exception as e:
        print(f"❌ Spline error: {e}")
        return waypoints


def compute_path_curvature(path_points):
    """곡률 계산"""
    if len(path_points) < 3:
        return np.zeros(len(path_points))
    
    curvatures = []
    for i in range(len(path_points)):
        if i == 0 or i == len(path_points) - 1:
            curvatures.append(0.0)
        else:
            p0 = path_points[i-1]
            p1 = path_points[i]
            p2 = path_points[i+1]
            
            v1 = p1 - p0
            v2 = p2 - p1
            
            angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
            dist = np.linalg.norm(v1) + np.linalg.norm(v2)
            
            if dist > 0.001:
                curvatures.append(abs(angle) / (dist + 0.001))
            else:
                curvatures.append(0.0)
    
    return np.array(curvatures)