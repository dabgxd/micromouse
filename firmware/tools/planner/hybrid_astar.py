"""Hybrid A* trong không gian (x, y, theta) cho xe vi sai, có bán kính quay tối thiểu.

Mặc định chỉ đi tiến. Khi lập quỹ đạo cứu (xe kẹt trước vật cản) có thể bật thêm:
  - lùi theo cung (allow_reverse), bị phạt chi phí w_rev và phạt mỗi lần đổi chiều w_switch_mm,
  - xoay tại chỗ (allow_spin) từng bước spin_step_deg, chi phí w_spin_mm_per_rad.
Mỗi node lưu chiều chuyển động: +1 tiến, -1 lùi, 0 xoay tại chỗ.
"""
import heapq
import math

import numpy as np


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _soft_pen(d, soft):
    return np.clip((soft - d) / soft, 0.0, None) ** 2


class HybridAStar:
    def __init__(self, m, car, cfg_astar, cfg_clear, recovery=None):
        """recovery: None (chỉ đi tiến) hoặc dict cấu hình lùi/xoay (config.json -> "recovery")."""
        self.m, self.car = m, car
        self.c = cfg_astar
        self.cl = cfg_clear
        self.rc = recovery
        r = cfg_astar["r_min_mm"]
        self.kappas = [0.0, 1 / (2 * r), -1 / (2 * r), 1 / r, -1 / r]
        # Tâm trục bánh luôn cách vật cản >= nửa bề rộng xe -> ô có khoảng trống nhỏ hơn coi như bị chặn.
        self.h_free_mm = car.hw - 3.0
        self._build_primitives()

    @staticmethod
    def _arc(k, s):
        if abs(k) < 1e-9:
            return (s, 0.0, 0.0)
        th = k * s
        return (math.sin(th) / k, (1 - math.cos(th)) / k, th)

    def _build_primitives(self):
        L, n = self.c["step_mm"], self.c["substeps"]
        self.prims = []  # (kappa, sub-poses (n,3), chiều, góc xoay)
        for k in self.kappas:
            self.prims.append((k, np.array([self._arc(k, L * i / n) for i in range(1, n + 1)]), 1, 0.0))
        if self.rc is not None:
            for k in self.kappas:
                self.prims.append((k, np.array([self._arc(k, -L * i / n) for i in range(1, n + 1)]), -1, 0.0))
            a = math.radians(self.rc["spin_step_deg"])
            for sg in (1, -1):
                self.prims.append((0.0, np.array([(0.0, 0.0, sg * a * i / n) for i in range(1, n + 1)]), 0, a))
        self.all_sub = np.vstack([p[1] for p in self.prims])

    def valid_and_cost(self, poses):
        db, dw = self.car.clearance(self.m, poses)
        if db.min() < self.cl["box_hard_mm"] or dw.min() < self.cl["wall_hard_mm"]:
            return None
        pen = _soft_pen(db, self.cl["box_soft_mm"]) + _soft_pen(dw, self.cl["wall_soft_mm"])
        return float(pen.mean())

    def _heuristic_grid(self, goal_mask_fn):
        """Dijkstra 2D (8 hướng) từ vùng đích trên lưới thô, dùng làm heuristic."""
        m, res = self.m, self.c["xy_res_mm"]
        x0, y0 = m.polygon.min(axis=0)
        x1, y1 = m.polygon.max(axis=0)
        nx, ny = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
        gx, gy = np.meshgrid(x0 + np.arange(nx) * res, y0 + np.arange(ny) * res)
        db, dw = m.lookup(gx, gy)
        free = (db >= self.h_free_mm) & (dw >= self.h_free_mm)
        goal = free & goal_mask_fn(gx, gy)
        dist = np.full((ny, nx), np.inf)
        pq = []
        for iy, ix in zip(*np.nonzero(goal)):
            dist[iy, ix] = 0.0
            pq.append((0.0, iy, ix))
        heapq.heapify(pq)
        nbr = [(dy, dx, res * math.hypot(dx, dy)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy]
        while pq:
            d, iy, ix = heapq.heappop(pq)
            if d > dist[iy, ix]:
                continue
            for dy, dx, c in nbr:
                jy, jx = iy + dy, ix + dx
                if 0 <= jy < ny and 0 <= jx < nx and free[jy, jx] and d + c < dist[jy, jx]:
                    dist[jy, jx] = d + c
                    heapq.heappush(pq, (d + c, jy, jx))
        self.h_grid, self.h_x0, self.h_y0, self.h_res = dist, x0, y0, res

    def h(self, x, y):
        ix = int(round((x - self.h_x0) / self.h_res))
        iy = int(round((y - self.h_y0) / self.h_res))
        ny, nx = self.h_grid.shape
        if 0 <= iy < ny and 0 <= ix < nx:
            return self.h_grid[iy, ix]
        return math.inf

    def _key(self, x, y, th):
        r, nb = self.c["xy_res_mm"], self.c["theta_bins"]
        return (int(math.floor(x / r)), int(math.floor(y / r)),
                int(round((th % (2 * math.pi)) / (2 * math.pi) * nb)) % nb)

    def plan_to_finish(self, starts, goal_y, goal_heading, heading_tol):
        fl = self.m.finish_line

        def goal_fn(x, y, th):
            return (y <= goal_y and fl["x_min"] <= x <= fl["x_max"]
                    and abs(_wrap(th - goal_heading)) <= heading_tol)

        def goal_mask(gx, gy):
            return (gy <= goal_y) & (gx >= fl["x_min"]) & (gx <= fl["x_max"])

        return self.plan(starts, goal_fn, goal_mask)

    def plan(self, starts, goal_fn, goal_mask_fn, max_expand=400000, check_start=True, h_weight=1.0):
        """starts: list (x, y, th). goal_fn(x, y, th) -> bool. goal_mask_fn(gx, gy) -> mảng bool cho heuristic.

        h_weight > 1: weighted A* (nhanh hơn, quỹ đạo có thể dài hơn tối ưu tối đa h_weight lần).
        Trả về (path, số node đã mở); path có các cột x, y, th, kappa, chiều (+1/-1/0).
        """
        self._heuristic_grid(goal_mask_fn)
        nodes = []  # (x, y, th, kappa, parent_idx, g, chiều, chiều di chuyển gần nhất)
        pq, best = [], {}
        for (x, y, th) in starts:
            if check_start and self.valid_and_cost(np.array([[x, y, th]])) is None:
                continue
            if not math.isfinite(self.h(x, y)):
                continue
            nodes.append((x, y, th, 0.0, -1, 0.0, 0, 0))
            best[self._key(x, y, th)] = 0.0
            heapq.heappush(pq, (h_weight * self.h(x, y), len(nodes) - 1))
        if not pq:
            raise RuntimeError("Không có vị trí xuất phát hợp lệ")

        L = self.c["step_mm"]
        r_min = self.c["r_min_mm"]
        ns = self.c["substeps"]
        rc = self.rc
        closed = set()
        expanded = 0
        while pq:
            _, idx = heapq.heappop(pq)
            x, y, th, kp, _, g, _, last_move = nodes[idx]
            key = self._key(x, y, th)
            if key in closed:
                continue
            closed.add(key)
            expanded += 1
            if expanded > max_expand:
                raise RuntimeError("Hybrid A* vượt quá số node cho phép")
            if goal_fn(x, y, th):
                return self._extract(nodes, idx), expanded

            c, s = math.cos(th), math.sin(th)
            # kiểm tra va chạm cho tất cả primitive trong một lần gọi numpy
            wx = x + c * self.all_sub[:, 0] - s * self.all_sub[:, 1]
            wy = y + s * self.all_sub[:, 0] + c * self.all_sub[:, 1]
            wth = th + self.all_sub[:, 2]
            db, dw = self.car.clearance(self.m, np.column_stack([wx, wy, wth]))
            for p, (k, _, d, ang) in enumerate(self.prims):
                sl = slice(p * ns, (p + 1) * ns)
                if db[sl].min() < self.cl["box_hard_mm"] or dw[sl].min() < self.cl["wall_hard_mm"]:
                    continue
                pen = float((_soft_pen(db[sl], self.cl["box_soft_mm"])
                             + _soft_pen(dw[sl], self.cl["wall_soft_mm"])).mean())
                nx_, ny_, nth = wx[sl][-1], wy[sl][-1], _wrap(wth[sl][-1])
                nkey = self._key(nx_, ny_, nth)
                if nkey in closed:
                    continue
                if d == 0:
                    step = rc["w_spin_mm_per_rad"] * ang + L * self.c["w_soft"] * pen
                    move = last_move
                else:
                    step = L * ((1 if d > 0 else rc["w_rev"]) + self.c["w_curv"] * abs(k) * r_min
                                + self.c["w_dcurv"] * abs(k - kp) * r_min + self.c["w_soft"] * pen)
                    if last_move and d != last_move:
                        step += rc["w_switch_mm"]
                    move = d
                ng = g + step
                if ng >= best.get(nkey, math.inf):
                    continue
                hh = self.h(nx_, ny_)
                if not math.isfinite(hh):
                    continue
                best[nkey] = ng
                nodes.append((nx_, ny_, nth, k, idx, ng, d, move))
                heapq.heappush(pq, (ng + h_weight * hh, len(nodes) - 1))
        raise RuntimeError("Hybrid A* không tìm được đường")

    def _extract(self, nodes, idx):
        path = []
        while idx >= 0:
            x, y, th, k, parent, _, d, _ = nodes[idx]
            path.append((x, y, th, k, d))
            idx = parent
        return np.array(path[::-1])
