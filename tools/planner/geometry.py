"""Mô hình bản đồ, trường khoảng cách và đường bao thân xe."""
import copy
import json
import math

import numpy as np
from matplotlib.path import Path
from scipy.ndimage import map_coordinates

FIELD_RES_MM = 5.0
FIELD_PAD_MM = 150.0


def _seg_dist(px, py, ax, ay, bx, by):
    """Khoảng cách từ các điểm (px, py) tới đoạn AB."""
    dx, dy = bx - ax, by - ay
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _rect_sdf(px, py, cx, cy, half):
    """Signed distance tới hình vuông tâm (cx, cy): dương bên ngoài, âm bên trong."""
    qx = np.abs(px - cx) - half
    qy = np.abs(py - cy) - half
    outside = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0))
    inside = np.minimum(np.maximum(qx, qy), 0)
    return outside + inside


class Map:
    def __init__(self, path):
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        self.tile = d["tile_mm"]
        self.box = d["box_mm"]
        self.tiles = d["tiles"]
        self.parts = d["parts_per_tile"]
        self.polygon = np.array(d["allowed_polygon"], dtype=float)
        self.boxes = [(b["id"], float(b["x"]), float(b["y"])) for b in d["static_boxes"]]
        self.finish_line = d["finish_line"]
        self.random_tiles = d["random_box_tiles"]
        self._build_fields()

    def wall_sdf(self, px, py):
        """Khoảng cách tới biên vùng được phép: dương bên trong, âm bên ngoài."""
        poly = self.polygon
        d = np.full(px.shape, np.inf)
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            d = np.minimum(d, _seg_dist(px, py, a[0], a[1], b[0], b[1]))
        inside = Path(poly).contains_points(np.column_stack([px.ravel(), py.ravel()]))
        return np.where(inside.reshape(px.shape), d, -d)

    def box_sdf(self, px, py, boxes=None, size=None):
        boxes = self.boxes if boxes is None else boxes
        half = (self.box if size is None else size) / 2
        d = np.full(px.shape, np.inf)
        for _, cx, cy in boxes:
            d = np.minimum(d, _rect_sdf(px, py, cx, cy, half))
        return d

    def _build_fields(self):
        x0, y0 = self.polygon.min(axis=0) - FIELD_PAD_MM
        x1, y1 = self.polygon.max(axis=0) + FIELD_PAD_MM
        self.fx0, self.fy0 = x0, y0
        xs = np.arange(x0, x1 + FIELD_RES_MM, FIELD_RES_MM)
        ys = np.arange(y0, y1 + FIELD_RES_MM, FIELD_RES_MM)
        gx, gy = np.meshgrid(xs, ys)  # chỉ số [iy, ix]
        self.gx, self.gy = gx, gy
        self.f_wall = self.wall_sdf(gx, gy)
        self.f_box = self.box_sdf(gx, gy)

    def with_extra_boxes(self, extra, inflate=0.0):
        """Bản sao map có thêm box (vd box ngẫu nhiên). extra: list (id, x, y).

        inflate: phóng to box thêm mỗi phía (mm) để bù độ bất định của vị trí ước lượng.
        """
        m = copy.copy(self)
        m.boxes = list(self.boxes) + list(extra)
        m.f_box = np.minimum(self.f_box, self.box_sdf(self.gx, self.gy, extra, self.box + 2 * inflate))
        return m

    def in_tiles(self, x, y, tiles=None):
        tiles = self.random_tiles if tiles is None else tiles
        return any(t["id"] in tiles and t["x0"] - 1e-6 <= x <= t["x0"] + self.tile + 1e-6
                   and t["y0"] - 1e-6 <= y <= t["y0"] + self.tile + 1e-6 for t in self.tiles)

    def box_fits(self, x, y, tiles=None):
        """Box tâm (x, y) (vị trí bất kỳ) nằm gọn trong vùng các ô cho phép và không chồng box tĩnh."""
        h = self.box / 2
        pts = [(x + sx * h, y + sy * h) for sx in (-1, 0, 1) for sy in (-1, 0, 1)]
        inside = all(self.in_tiles(px, py, tiles) for px, py in pts)
        overlap = any(abs(x - bx) < self.box and abs(y - by) < self.box for _, bx, by in self.boxes)
        return inside and not overlap

    def lookup(self, px, py):
        """Tra (d_box, d_wall) tại các điểm bằng nội suy song tuyến."""
        coords = np.vstack([(np.ravel(py) - self.fy0) / FIELD_RES_MM,
                            (np.ravel(px) - self.fx0) / FIELD_RES_MM])
        db = map_coordinates(self.f_box, coords, order=1, cval=-1e3)
        dw = map_coordinates(self.f_wall, coords, order=1, cval=-1e3)
        shape = np.shape(px)
        return db.reshape(shape), dw.reshape(shape)

    def tile_of(self, x, y):
        for t in self.tiles:
            if t["x0"] <= x < t["x0"] + self.tile and t["y0"] <= y < t["y0"] + self.tile:
                return t
        return None


class Car:
    """Đường bao chữ nhật của xe, gốc tại tâm trục bánh."""

    def __init__(self, cfg, spacing_mm=8.0):
        self.front = cfg["front_mm"]
        self.rear = cfg["rear_mm"]
        self.hw = cfg["half_width_mm"]
        r, b, w = cfg["wheel_diameter_mm"] / 2, cfg["track_mm"] / 2, cfg.get("wheel_width_mm", 18) / 2
        self.wheels = [np.array([[r, sy * b + w], [-r, sy * b + w], [-r, sy * b - w], [r, sy * b - w]])
                       for sy in (1, -1)]
        self.corners = np.array([[self.front, self.hw], [-self.rear, self.hw],
                                 [-self.rear, -self.hw], [self.front, -self.hw]])
        pts = []
        for i in range(4):
            a, b = self.corners[i], self.corners[(i + 1) % 4]
            n = max(2, int(math.ceil(np.linalg.norm(b - a) / spacing_mm)))
            for t in np.linspace(0, 1, n, endpoint=False):
                pts.append(a + t * (b - a))
        self.perimeter = np.array(pts)

    def world(self, poses, local):
        """poses (N,3) -> toạ độ thế giới (N, M, 2) của các điểm local (M,2)."""
        poses = np.atleast_2d(poses)
        c = np.cos(poses[:, 2])[:, None]
        s = np.sin(poses[:, 2])[:, None]
        wx = poses[:, 0:1] + c * local[:, 0] - s * local[:, 1]
        wy = poses[:, 1:2] + s * local[:, 0] + c * local[:, 1]
        return wx, wy

    def clearance(self, m, poses):
        """Khoảng cách nhỏ nhất (d_box, d_wall) từ thân xe tới vật cản / biên, cho từng pose."""
        wx, wy = self.world(poses, self.perimeter)
        db, dw = m.lookup(wx, wy)
        return db.min(axis=1), dw.min(axis=1)

    def clearance_to(self, poses, boxes, box_mm=100.0):
        """Khoảng cách chính xác (giải tích) từ thân xe tới một danh sách box (id, x, y)."""
        wx, wy = self.world(poses, self.perimeter)
        d = np.full(wx.shape, np.inf)
        for _, cx, cy in boxes:
            d = np.minimum(d, _rect_sdf(wx, wy, cx, cy, box_mm / 2))
        return d.min(axis=1)
