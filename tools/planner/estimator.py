"""Thuật toán chạy trên xe (bản Python để thử trước khi viết C):
odometry + gyro, EKF dùng ToF đối chiếu box đã biết, phát hiện box lạ.
"""
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Ekf:
    def __init__(self, pose0, cfg_ekf, tof, known_boxes, p0_xy_mm=5.0, p0_th_deg=1.5):
        self.x = np.array(pose0, dtype=float)
        self.P = np.diag([p0_xy_mm ** 2, p0_xy_mm ** 2, math.radians(p0_th_deg) ** 2])
        self.c = cfg_ekf
        self.tof = tof
        self.known = known_boxes
        self.max_r = cfg_ekf["max_range_mm"]
        self.max_inc = math.radians(cfg_ekf["max_incidence_deg"])

    def predict(self, ds, dth, dt):
        x, y, th = self.x
        thm = th + dth / 2
        c, s = math.cos(thm), math.sin(thm)
        self.x = np.array([x + ds * c, y + ds * s, wrap(th + dth)])
        F = np.array([[1, 0, -ds * s], [0, 1, ds * c], [0, 0, 1]])
        G = np.array([[c, -0.5 * ds * s], [s, 0.5 * ds * c], [0, 1]])
        sd = self.c["q_dist_rel"] * abs(ds) + 1e-3
        # Sai số hệ số gyro là sai số HỆ THỐNG: sau một góc quay A, độ lệch ~ q*A (không phải q*A/sqrt(n)).
        # Phương sai tăng tỉ lệ |dth| với góc tham chiếu pi/2 -> sau khi quay pi/2 thì sigma = q*pi/2.
        var_th = (math.radians(self.c["q_gyro_dps"]) * dt) ** 2 \
            + self.c["q_gyro_scale"] ** 2 * abs(dth) * (math.pi / 2)
        self.P = F @ self.P @ F.T + G @ np.diag([sd * sd, var_th]) @ G.T

    def update_tof(self, i, r, sigma_rel, sigma_mm):
        """Trả về ('acc'|'rej'|'none', innovation)."""
        pred = self.tof.predict(self.x, i, self.known, self.max_r, self.max_inc)
        if pred is None:
            return "none", None
        h0 = pred[0]
        H = np.zeros(3)
        for k, eps in enumerate((1.0, 1.0, 1e-3)):
            xp = self.x.copy()
            xp[k] += eps
            p = self.tof.predict(xp, i, self.known, self.max_r + 50, self.max_inc + 0.1)
            if p is None:
                return "none", None
            H[k] = (p[0] - h0) / eps
        Rm = (sigma_rel * h0 + sigma_mm) ** 2
        S = H @ self.P @ H + Rm
        innov = r - h0
        if innov * innov / S > self.c["gate_chi2"]:
            return "rej", innov
        K = self.P @ H / S
        self.x = self.x + K * innov
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(3) - np.outer(K, H)) @ self.P
        return "acc", innov


def _sdf_box(px, py, cx, cy, h):
    qx = np.abs(px - cx) - h
    qy = np.abs(py - cy) - h
    return np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0)


class Detector:
    """Phát hiện box lạ và ước lượng tâm box ở VỊ TRÍ LIÊN TỤC (không theo lưới).

    Mỗi số đo "bất thường" (ngắn hơn hẳn so với map đã biết) cho một cung điểm ở khoảng cách r trong nón đo.
    Với VL53L0X (trả về mục tiêu gần nhất): ít nhất một điểm trên cung nằm trên mặt box, các điểm còn lại
    nằm ngoài box. Khi đủ số đo gần nhau -> khớp hình vuông 100 mm (tìm tâm làm cực tiểu sai lệch).
    """

    def __init__(self, m, cfg_det):
        self.c = cfg_det
        self.m = m
        self.static = [(x, y) for _, x, y in m.boxes]
        self.h = m.box / 2
        self.reads = []      # các số đo bất thường: dict(pts (5,2), c (2,), t)
        self.cluster = []    # các số đo thuộc box đang theo dõi
        self.est = None      # tâm box ước lượng (x, y)
        self.spread = math.inf
        self.t_fit = -1.0
        self.history = []    # (t, x, y) mỗi lần ước lượng

    def uncertainty(self):
        """Độ bất định vị trí box (mm mỗi phía) để phóng to box khi lập quỹ đạo: lớn khi mới có ít số đo."""
        if self.est is None:
            return 0.0
        n = max(1, len(self.cluster))
        return float(min(self.c["inflate_max_mm"], self.spread + self.c["inflate_k_mm"] / math.sqrt(n)))

    def _fits(self, cx, cy):
        """Bản vector hoá của Map.box_fits."""
        m, h = self.m, self.h
        tiles = [t for t in m.tiles if t["id"] in m.random_tiles]
        ok = np.ones(cx.shape, bool)
        for sx in (-1, 0, 1):
            for sy in (-1, 0, 1):
                px, py = cx + sx * h, cy + sy * h
                inside = np.zeros(cx.shape, bool)
                for t in tiles:
                    inside |= ((px >= t["x0"] - 1e-6) & (px <= t["x0"] + m.tile + 1e-6)
                               & (py >= t["y0"] - 1e-6) & (py <= t["y0"] + m.tile + 1e-6))
                ok &= inside
        for bx, by in self.static:
            ok &= ~((np.abs(cx - bx) < m.box) & (np.abs(cy - by) < m.box))
        return ok

    def _plausible(self, px, py):
        if not self.m.in_tiles(px, py):
            return False
        # điểm rơi vào (hoặc sát) box tĩnh -> do sai số định vị, bỏ qua
        return not any(abs(px - bx) < self.h + 20 and abs(py - by) < self.h + 20 for bx, by in self.static)

    def add(self, sx, sy, sa, r, offsets, t):
        """Trả về ('new' | 'update', (x, y)) hoặc None."""
        dirs = np.column_stack([np.cos(sa + offsets), np.sin(sa + offsets)])
        pts = np.array([sx, sy]) + r * dirs
        inner = pts + 25 * dirs
        ok = [self._plausible(px, py) for px, py in inner]
        if not any(ok):
            return None
        # điểm trên các tia TRƯỚC khoảng cách đo -> chắc chắn trống
        free = np.concatenate([np.array([sx, sy]) + f * r * dirs for f in (0.3, 0.6, 0.85)])
        rd = {"pts": pts, "free": free, "c": inner[ok].mean(axis=0), "t": t, "o": np.array([sx, sy])}
        max_spread = self.c.get("max_spread_mm", 30.0)

        # Số đo thuộc box đang theo dõi: tinh chỉnh ước lượng.
        if self.est is not None and np.hypot(*(rd["c"] - np.array(self.est))) < 1.5 * self.m.box:
            self.cluster.append(rd)
            if t - self.t_fit < 0.1:  # khớp lại tối đa 10 lần/giây
                return None
            self.t_fit = t
            fit = self.fit(self.cluster, self.est)
            if fit is not None and fit[2] <= max(max_spread, self.spread):
                self.est, self.spread = fit[:2], fit[2]
                self.history.append((t, *self.est))
                return "update", self.est
            return None

        # Số đo không thuộc box đang theo dõi: gom cụm mới. Chỉ có MỘT box ngẫu nhiên, nên nếu cụm mới
        # đủ tin cậy thì nó thay thế ước lượng cũ (ước lượng cũ có thể sai do nhìn từ quá xa).
        # Khi box đã được xác nhận bằng nhiều số đo thì khoá lại: mọi cụm khác là nhiễu.
        if self.est is not None and len(self.cluster) >= self.c["lock_reads"]:
            return None
        self.reads.append(rd)
        win = 2 * self.c["decay_s"]
        self.reads = [q for q in self.reads if t - q["t"] < win]
        near = [q for q in self.reads if np.hypot(*(q["c"] - rd["c"])) < 1.5 * self.m.box]
        baseline = np.ptp(np.array([q["o"] for q in near]), axis=0).max() if len(near) > 1 else 0.0
        # thay thế ước lượng đã có cần nhiều bằng chứng hơn (tránh cụm giả do nhiễu đột biến của ToF)
        need = self.c["hit_thresh"] * (1 if self.est is None else 2)
        if len(near) >= need and baseline >= 20:
            fit = self.fit(near, tuple(np.mean([q["c"] for q in near], axis=0)))
            # chỉ xác nhận khi đủ chắc chắn, hoặc đã có rất nhiều số đo
            if fit is not None and (fit[2] <= max_spread or len(near) >= 3 * self.c["hit_thresh"]):
                kind = "new" if self.est is None else "update"
                self.est, self.spread, self.cluster = fit[:2], fit[2], near
                used = {id(q) for q in near}
                self.reads = [q for q in self.reads if id(q) not in used]
                self.history.append((t, *self.est))
                return kind, self.est
        return None

    def _cost(self, P, F, cx, cy, tol=8.0):
        X, Y = cx[:, None, None], cy[:, None, None]
        d = _sdf_box(P[None, :, :, 0], P[None, :, :, 1], X, Y, self.h)    # (K, R, 5)
        df = _sdf_box(F[None, :, :, 0], F[None, :, :, 1], X, Y, self.h)  # (K, R, 15)
        pen = (np.clip(-d - tol, 0, None) ** 2).sum(axis=2) + (np.clip(-df - tol, 0, None) ** 2).sum(axis=2)
        per = np.minimum(d.min(axis=2) ** 2 + 4 * pen, 40.0 ** 2)   # (K, R) có chặn trên
        return np.where(self._fits(cx, cy), per.sum(axis=1), np.inf)

    def fit(self, reads, guess, span=160.0, max_reads=30):
        """Tìm tâm box khớp nhất với các số đo (tìm thô bước 10 mm rồi tinh chỉnh bước 2.5 mm).

        Mỗi số đo: điểm gần nhất trên cung phải nằm trên mặt box, không điểm nào của cung hoặc
        của phần tia phía trước được nằm trong box. Sai số mỗi số đo bị chặn trên (chống nhiễu đột biến).
        Trả về (x, y, spread): spread = bán kính vùng các tâm gần tối ưu (độ bất định).
        """
        reads = reads[-max_reads:]
        P = np.stack([q["pts"] for q in reads])   # (R, 5, 2)
        F = np.stack([q["free"] for q in reads])  # (R, 15, 2)
        g = np.arange(-span, span + 10, 10.0)
        cx = (guess[0] + g[:, None] + 0 * g[None, :]).ravel()
        cy = (guess[1] + 0 * g[:, None] + g[None, :]).ravel()
        cost = self._cost(P, F, cx, cy)
        if not np.isfinite(cost).any():
            return None
        i = int(np.argmin(cost))
        near = cost <= cost[i] + len(reads) * 6.0 ** 2
        spread = float(np.hypot(cx[near] - cx[i], cy[near] - cy[i]).max())
        g2 = np.arange(-12.5, 12.6, 2.5)
        fx = (cx[i] + g2[:, None] + 0 * g2[None, :]).ravel()
        fy = (cy[i] + 0 * g2[:, None] + g2[None, :]).ravel()
        c2 = self._cost(P, F, fx, fy)
        j = int(np.argmin(c2))
        return float(fx[j]), float(fy[j]), spread
