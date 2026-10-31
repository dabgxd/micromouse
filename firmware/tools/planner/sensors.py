"""Mô hình ToF VL53L0X: bắn nhiều tia trong nón đo vào các box (AABB)."""
import math

import numpy as np

INF = float("inf")


def ray_box(ox, oy, dx, dy, cx, cy, h):
    """Giao tia (o, d) với hình vuông tâm (cx, cy) nửa cạnh h. Trả về (t, nx, ny) mặt đi vào, hoặc None."""
    tmin, tmax = -INF, INF
    nx = ny = 0.0
    if abs(dx) < 1e-12:
        if ox < cx - h or ox > cx + h:
            return None
    else:
        t1, t2 = (cx - h - ox) / dx, (cx + h - ox) / dx
        n = -1.0
        if t1 > t2:
            t1, t2, n = t2, t1, 1.0
        tmin, tmax, nx = t1, t2, n
    if abs(dy) < 1e-12:
        if oy < cy - h or oy > cy + h:
            return None
    else:
        t1, t2 = (cy - h - oy) / dy, (cy + h - oy) / dy
        n = -1.0
        if t1 > t2:
            t1, t2, n = t2, t1, 1.0
        if t1 > tmin:
            tmin, nx, ny = t1, 0.0, n
        tmax = min(tmax, t2)
    if tmin < 0 or tmax < tmin:
        return None
    return tmin, nx, ny


def raycast(ox, oy, ang, boxes, h, max_r):
    """Tia gần nhất. boxes: list (x, y). Trả về (t, idx, nx, ny) hoặc None."""
    dx, dy = math.cos(ang), math.sin(ang)
    best = None
    for i, (cx, cy) in enumerate(boxes):
        # loại nhanh box ở quá xa hoặc phía sau
        rx, ry = cx - ox, cy - oy
        if rx * dx + ry * dy < -h * 1.5 or rx * rx + ry * ry > (max_r + 1.5 * h) ** 2:
            continue
        r = ray_box(ox, oy, dx, dy, cx, cy, h)
        if r is not None and r[0] <= max_r and (best is None or r[0] < best[0]):
            best = (r[0], i, r[1], r[2])
    return best


class Tof:
    def __init__(self, robot_cfg, box_mm, n_rays=5):
        self.names = [t["name"] for t in robot_cfg["tof"]]
        self.mounts = [(t["x"], t["y"], math.radians(t["deg"])) for t in robot_cfg["tof"]]
        fov = math.radians(robot_cfg["tof_fov_deg"])
        self.offsets = np.linspace(-fov / 2, fov / 2, n_rays)
        self.max_r = robot_cfg["tof_max_mm"]
        self.h = box_mm / 2

    def sensor_pose(self, pose, i):
        x, y, th = pose
        mx, my, ma = self.mounts[i]
        c, s = math.cos(th), math.sin(th)
        return x + c * mx - s * my, y + s * mx + c * my, th + ma

    def true_range(self, pose, i, boxes):
        """Khoảng cách thật = tia ngắn nhất trong nón (VL53L0X trả về mục tiêu gần nhất)."""
        sx, sy, sa = self.sensor_pose(pose, i)
        best = None
        for off in self.offsets:
            r = raycast(sx, sy, sa + off, boxes, self.h, self.max_r)
            if r is not None and (best is None or r[0] < best):
                best = r[0]
        return best

    def predict(self, pose, i, boxes, max_r, max_inc):
        """Dự đoán số đo từ map đã biết. Chỉ trả về khi cả nón rơi trên CÙNG một mặt box và góc tới đủ nhỏ."""
        sx, sy, sa = self.sensor_pose(pose, i)
        face, best = None, INF
        for off in self.offsets:
            r = raycast(sx, sy, sa + off, boxes, self.h, max_r)
            if r is None:
                return None
            f = (r[1], r[2], r[3])
            if face is None:
                face = f
            elif f != face:
                return None
            best = min(best, r[0])
        # góc tới của tia giữa so với pháp tuyến mặt
        inc = math.acos(min(1.0, abs(math.cos(sa) * face[1] + math.sin(sa) * face[2])))
        if inc > max_inc:
            return None
        return best, face[0]
