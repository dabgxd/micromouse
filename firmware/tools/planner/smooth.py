"""Làm mượt quỹ đạo bằng tối ưu (L-BFGS) và nội suy spline."""
import math

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize

# Mỗi số hạng chi phí tại điểm j chỉ phụ thuộc các điểm j-1..j+1, nên có thể
# tính gradient bằng sai phân "tô màu": dịch đồng thời các điểm cách nhau 5.
_COLORS = 5
_H = 0.05


def resample(xy, spacing):
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(3, int(round(s[-1] / spacing)) + 1)
    si = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(si, s, xy[:, 0]), np.interp(si, s, xy[:, 1])])


def _headings(P):
    d = np.empty_like(P)
    d[1:-1] = P[2:] - P[:-2]
    d[0] = P[1] - P[0]
    d[-1] = P[-1] - P[-2]
    return np.arctan2(d[:, 1], d[:, 0])


def _curvature(P):
    k = np.zeros(len(P))
    a, b, c = P[:-2], P[1:-1], P[2:]
    ab, bc, ac = (np.hypot(*(b - a).T), np.hypot(*(c - b).T), np.hypot(*(c - a).T))
    cross = (b - a)[:, 0] * (c - b)[:, 1] - (b - a)[:, 1] * (c - b)[:, 0]
    k[1:-1] = 2 * cross / np.maximum(ab * bc * ac, 1e-9)
    return k


class Smoother:
    def __init__(self, m, car, cfg_smooth, cfg_clear, r_min):
        self.m, self.car, self.c, self.cl = m, car, cfg_smooth, cfg_clear
        self.k_max = 1.0 / r_min

    def terms(self, P, P0):
        c, cl = self.c, self.cl
        n = len(P)
        t = np.zeros(n)
        sp = self.c["spacing_mm"]
        lap = np.zeros_like(P)
        lap[1:-1] = P[2:] - 2 * P[1:-1] + P[:-2]
        t += c["w_smooth"] * (lap ** 2).sum(axis=1) / sp ** 2

        poses = np.column_stack([P, _headings(P)])
        db, dw = self.car.clearance(self.m, poses)
        for d, soft, hard in ((db, cl["box_soft_mm"], cl["box_hard_mm"]),
                              (dw, cl["wall_soft_mm"], cl["wall_hard_mm"])):
            t += c["w_soft"] * (np.clip(soft - d, 0, None) / soft) ** 2
            t += c["w_hard"] * (np.clip(hard + 5 - d, 0, None) / 10) ** 2

        k = np.abs(_curvature(P))
        t += c["w_curv"] * (np.clip(k - 0.8 * self.k_max, 0, None) / self.k_max) ** 2
        t += c["w_dev"] * ((P - P0) ** 2).sum(axis=1)
        return t

    def run(self, path_xy):
        P0 = resample(path_xy, self.c["spacing_mm"])
        n = len(P0)
        free = np.arange(2, n - 2)  # giữ cố định 2 điểm đầu/cuối để giữ nguyên hướng

        def fun(v):
            P = P0.copy()
            P[free] = v.reshape(-1, 2)
            base = self.terms(P, P0)
            g = np.zeros((n, 2))
            for col in range(_COLORS):
                idx = free[free % _COLORS == col]
                for dim in range(2):
                    Q = P.copy()
                    Q[idx, dim] += _H
                    diff = self.terms(Q, P0) - base
                    win = np.convolve(diff, np.ones(3), mode="same")
                    g[idx, dim] = win[idx] / _H
            return base.sum(), g[free].ravel()

        res = minimize(fun, P0[free].ravel(), jac=True, method="L-BFGS-B",
                       options={"maxiter": self.c["iters"]})
        P = P0.copy()
        P[free] = res.x.reshape(-1, 2)
        return P


def spline_sample(P, step):
    """Nội suy spline theo độ dài cung, trả về mảng [s, x, y, theta, kappa]."""
    seg = np.hypot(*np.diff(P, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    sx, sy = CubicSpline(s, P[:, 0]), CubicSpline(s, P[:, 1])
    u = np.arange(0, s[-1], step / 4)
    # tính lại độ dài cung thực của spline rồi lấy mẫu đều
    dx, dy = sx(u, 1), sy(u, 1)
    arc = np.concatenate([[0], np.cumsum(np.hypot(dx, dy)[:-1] * np.diff(u))])
    so = np.arange(0, arc[-1], step)
    uo = np.interp(so, arc, u)
    x, y = sx(uo), sy(uo)
    dx, dy, ddx, ddy = sx(uo, 1), sy(uo, 1), sx(uo, 2), sy(uo, 2)
    th = np.arctan2(dy, dx)
    k = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-9)
    return np.column_stack([so, x, y, th, k])


def speed_profile(traj, cfg):
    """v(s) [m/s] giới hạn bởi gia tốc ngang, tăng tốc và giảm tốc. Trả về (v, thời gian)."""
    s = traj[:, 0] / 1000.0
    k = np.abs(traj[:, 4]) * 1000.0
    v = np.minimum(cfg["v_max_mps"], np.sqrt(cfg["a_lat_mps2"] / np.maximum(k, 1e-6)))
    v[0] = 0.0
    v[-1] = 0.0
    ds = np.diff(s)
    for i in range(1, len(v)):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * cfg["a_acc_mps2"] * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * cfg["a_dec_mps2"] * ds[i]))
    vm = np.maximum((v[1:] + v[:-1]) / 2, 1e-3)
    t = np.concatenate([[0], np.cumsum(ds / vm)])
    return v, t
