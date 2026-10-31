"""Vạch chia phần (24 vạch), thời điểm cả xe qua vạch, gán box vào phần, bảng chỉnh xe."""
import math

import numpy as np


class Line:
    def __init__(self, k, tile, a, b, n):
        self.k, self.tile = k, tile
        self.a, self.b = np.array(a, float), np.array(b, float)
        self.n = np.array(n, float) / np.linalg.norm(n)  # hướng đi qua vạch
        self.u = (self.b - self.a) / np.linalg.norm(self.b - self.a)


def build_lines(m):
    lines, k = [], 1
    T = m.tile
    for t in m.tiles:
        x0, y0 = t["x0"], t["y0"]
        if t["type"] == "straight":
            n = m.parts["straight"]
            d = np.array(t["dir"], float)
            for i in range(1, n + 1):
                f = i / n
                if d[1] > 0:
                    y = y0 + f * T; a, b = (x0, y), (x0 + T, y)
                elif d[1] < 0:
                    y = y0 + T - f * T; a, b = (x0 + T, y), (x0, y)
                else:
                    x = x0 + f * T; a, b = (x, y0 + T), (x, y0)
                lines.append(Line(k, t["id"], a, b, d)); k += 1
        else:
            n = m.parts["corner"]
            c = np.array(t["inner"], float)
            a0, a1 = t["angles_deg"]
            for i in range(1, n + 1):
                phi = math.radians(a0 + (a1 - a0) * i / n)
                u = np.array([math.cos(phi), math.sin(phi)])
                # vạch kéo từ góc trong tới biên ngoài của ô
                r = T / max(abs(u[0]), abs(u[1]))
                normal = np.array([math.sin(phi), -math.cos(phi)])  # đi theo chiều kim đồng hồ quanh góc trong
                lines.append(Line(k, t["id"], c, c + r * u, normal)); k += 1
    return lines


def car_corners(car, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.array([[x + c * px - s * py, y + s * px + c * py] for px, py in car.corners])


def crossing_index(car, traj, lines):
    """Chỉ số mẫu đầu tiên mà cả 4 góc xe đã qua từng vạch (xét tuần tự)."""
    out, i0 = [], 0
    for ln in lines:
        found = None
        for i in range(i0, len(traj)):
            cs = car_corners(car, traj[i, 1:4])
            if np.all((cs - ln.a) @ ln.n > 0):
                found = i
                break
        out.append(found)
        if found is not None:
            i0 = found
    return out


def box_partition(m, lines, bx, by):
    """Vạch kết thúc của phần chứa box (qua vạch này thì được cộng điểm box)."""
    t = m.tile_of(bx, by)
    for ln in lines:
        if ln.tile != t["id"]:
            continue
        if np.dot(np.array([bx, by]) - ln.a, ln.n) < 0:
            return ln.k
    return None


def _placed_pose(car, ln, along):
    p = ln.a + along * ln.u + 2.0 * ln.n  # đuôi xe cách vạch 2 mm
    axle = p + car.rear * ln.n
    return (float(axle[0]), float(axle[1]), math.atan2(ln.n[1], ln.n[0]))


def recovery_table(m, car, traj, lines, cross, cl):
    """Với mỗi vạch k: đặt đuôi xe sát vạch, thân xe song song làn.

    Vị trí ngang lấy theo quỹ đạo; nếu ở đó xe sát vật cản thì dịch tới vị trí an toàn gần nhất.
    """
    rows = []
    for ln, idx in zip(lines, cross):
        if idx is None:
            rows.append(None)
            continue
        x, y, th = traj[idx, 1:4]
        rear_c = np.array([x, y]) - car.rear * np.array([math.cos(th), math.sin(th)])
        along0 = float(np.dot(rear_c - ln.a, ln.u))  # khoảng cách từ đầu vạch (ln.a) tới tâm xe
        cands = along0 + np.arange(-200, 201, 5.0)
        cands = cands[np.argsort(np.abs(cands - along0))]
        poses = np.array([_placed_pose(car, ln, a) for a in cands])
        db, dw = car.clearance(m, poses)
        good = np.nonzero((db >= cl["box_soft_mm"]) & (dw >= cl["wall_soft_mm"]))[0]
        j = good[0] if len(good) else int(np.argmax(np.minimum(db - cl["box_hard_mm"], dw - cl["wall_hard_mm"])))
        rows.append({"k": ln.k, "tile": ln.tile, "along_mm": float(cands[j]),
                     "pose": tuple(poses[j]), "traj_idx": idx,
                     "clear_box": float(db[j]), "clear_wall": float(dw[j])})
    return rows


def describe_line(m, ln):
    """Mô tả mốc đo ngang cho người đặt xe."""
    if ln.tile in (2, 4):
        return "đo dọc vạch chéo, từ góc trong chữ U ra ngoài"
    d = ln.n
    if d[1] > 0:
        return "đo từ biên TRÁI (x=0) sang phải"
    if d[0] > 0:
        return "đo từ biên TRÊN (y=1200) xuống dưới"
    return "đo từ biên PHẢI (x=1800) sang trái"
