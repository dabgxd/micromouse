"""Các bước lập quỹ đạo dùng chung cho plan.py và sim_gui.py."""
import hashlib
import json
import math
import os

import numpy as np
from scipy.spatial import cKDTree

from geometry import Car, Map
from hybrid_astar import HybridAStar, _wrap
from smooth import Smoother, speed_profile, spline_sample

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


class Context:
    def __init__(self):
        with open(os.path.join(HERE, "config.json"), encoding="utf-8") as f:
            self.cfg = json.load(f)
        self.m = Map(os.path.join(HERE, "map.json"))
        self.car = Car(self.cfg["robot"])
        self.cl = self.cfg["clearance"]

    def planning_hash(self):
        h = hashlib.sha1()
        with open(os.path.join(HERE, "map.json"), "rb") as f:
            h.update(f.read())
        keys = ("robot", "clearance", "start", "goal", "astar", "smooth", "output")
        h.update(json.dumps({k: self.cfg[k] for k in keys}, sort_keys=True).encode())
        return h.hexdigest()[:12]


def plan_nominal(ctx, m=None, log=print):
    """Quỹ đạo chính: Hybrid A* -> làm mượt -> spline. Trả về (traj, raw, ok)."""
    m = ctx.m if m is None else m
    cfg, car, cl = ctx.cfg, ctx.car, ctx.cl
    st = cfg["start"]
    starts = [(x, st["axle_y_mm"], math.pi / 2)
              for x in np.arange(st["x_min_mm"], st["x_max_mm"] + 1, st["x_step_mm"])]
    astar = HybridAStar(m, car, cfg["astar"], cl)
    raw, n_exp = astar.plan_to_finish(starts, cfg["goal"]["axle_y_max_mm"], -math.pi / 2,
                                      math.radians(cfg["goal"]["heading_tol_deg"]))
    log(f"[1] Hybrid A*: {len(raw)} node, đã mở {n_exp} node, xuất phát x = {raw[0, 0]:.0f} mm")

    sm = Smoother(m, car, cfg["smooth"], cl, cfg["astar"]["r_min_mm"])
    traj = spline_sample(sm.run(raw[:, :2]), cfg["output"]["sample_mm"])
    db, dw = car.clearance(m, traj[:, 1:4])
    ok = db.min() >= cl["box_hard_mm"] and dw.min() >= cl["wall_hard_mm"]
    k_max = np.abs(traj[:, 4]).max()
    log(f"[2] Làm mượt: cách box min {db.min():.1f} mm, cách biên min {dw.min():.1f} mm, "
        f"R min = {1 / max(k_max, 1e-9):.0f} mm")
    if k_max > 1.02 / cfg["astar"]["r_min_mm"]:
        log(f"    !! Bán kính cua nhỏ hơn r_min = {cfg['astar']['r_min_mm']} mm")
    if not ok:
        log("    !! Quỹ đạo sau làm mượt vi phạm biên an toàn cứng -> dùng quỹ đạo A* thô")
        traj = spline_sample(raw[:, :2], cfg["output"]["sample_mm"])
    return traj, raw, ok


def nominal_cached(ctx, log=print):
    """Quỹ đạo chính, lưu cache theo nội dung map.json + config.json."""
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "nominal_cache.npz")
    key = ctx.planning_hash()
    if os.path.exists(path):
        d = np.load(path)
        if str(d["key"]) == key:
            return d["traj"], d["raw"]
    traj, raw, _ = plan_nominal(ctx, log=log)
    np.savez(path, key=key, traj=traj, raw=raw)
    return traj, raw


def _recompute_geometry(xy):
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    return s


def _rejoin_goal(ctx, base, i_join_min, tol_mm=30.0, tol_deg=15.0):
    """Đích cho Hybrid A*: nhập lại quỹ đạo `base` tại mẫu >= i_join_min, HOẶC đi thẳng qua vạch đích."""
    tgt = base[i_join_min:len(base) - 5]
    tree = cKDTree(tgt[:, 1:3])
    tol_th = math.radians(tol_deg)
    gy_max = ctx.cfg["goal"]["axle_y_max_mm"]
    fl = ctx.m.finish_line

    def goal_fn(x, y, th):
        if y <= gy_max and fl["x_min"] <= x <= fl["x_max"] and abs(_wrap(th + math.pi / 2)) <= tol_th:
            return True
        d, j = tree.query((x, y))
        return d <= tol_mm and abs(_wrap(th - tgt[j, 3])) <= tol_th

    def goal_mask(gx, gy):
        d, _ = tree.query(np.column_stack([gx.ravel(), gy.ravel()]))
        fin = (gy <= gy_max) & (gx >= fl["x_min"]) & (gx <= fl["x_max"])
        return (d <= tol_mm).reshape(gx.shape) | fin

    return goal_fn, goal_mask, tree


def concat_traj(a, b):
    """Nối hai quỹ đạo [s, x, y, th, k] (bỏ điểm đầu của b nếu trùng), tính lại s."""
    if len(a) and len(b) and np.hypot(*(a[-1, 1:3] - b[0, 1:3])) < 1.0:
        b = b[1:]
    t = np.vstack([a, b])
    t[:, 0] = _recompute_geometry(t[:, 1:3])
    return t


def _segment_traj(xy, step, reverse, car, m, cl):
    """Quỹ đạo cho một đoạn tiến/lùi từ các node A*. th = hướng CHUYỂN ĐỘNG (khi lùi = hướng thân + pi)."""
    if len(xy) >= 3:
        tr = spline_sample(xy, step)
        body = tr[:, 1:4].copy()
        if reverse:
            body[:, 2] += math.pi
        db, dw = car.clearance(m, body)
        if db.min() >= cl["box_hard_mm"] and dw.min() >= cl["wall_hard_mm"]:
            return tr
    # nội suy tuyến tính giữa các node (an toàn vì node A* đã được kiểm tra va chạm)
    s = _recompute_geometry(xy)
    n = max(2, int(s[-1] / step) + 1)
    si = np.linspace(0, s[-1], n)
    x, y = np.interp(si, s, xy[:, 0]), np.interp(si, s, xy[:, 1])
    th = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    k = np.gradient(th) / np.maximum(np.gradient(si), 1e-6)
    return np.column_stack([si, x, y, th, k])


def plan_recovery(ctx, base, box, pose, idx, inflate=0.0):
    """Quỹ đạo cứu khi xe kẹt: được lùi và xoay tại chỗ, từ pose hiện tại tới quỹ đạo `base` (sau box) hoặc đích.

    Trả về dict: ok, segments = list ('spin', góc) | ('fwd' | 'rev', traj [s,x,y,th,k]).
    Đoạn tiến cuối cùng đã được nối với phần còn lại của `base`.
    """
    cfg, car = ctx.cfg, ctx.car
    rc = cfg["recovery"]
    step = cfg["output"]["sample_mm"]
    rbox = [("R", float(box[0]), float(box[1]))] if box is not None else []
    m2 = ctx.m.with_extra_boxes(rbox, inflate) if rbox else ctx.m
    db0, dw0 = car.clearance(m2, np.array([pose]))
    # Xe có thể đang đứng sát box hơn biên an toàn: nới biên cứng xuống bằng khoảng cách hiện tại.
    cl = dict(ctx.cl)
    cl["box_hard_mm"] = min(cl["box_hard_mm"], max(float(db0[0]) - 3.0, rc["min_clear_mm"]))
    cl["wall_hard_mm"] = min(cl["wall_hard_mm"], float(dw0[0]) - 3.0)
    if rbox:
        j_box = int(np.argmin((base[:, 1] - box[0]) ** 2 + (base[:, 2] - box[1]) ** 2))
        i_join_min = max(idx + 5, j_box + int(cfg["detect"]["join_ahead_mm"] / step))
    else:
        i_join_min = idx + int(300 / step)
    i_join_min = min(i_join_min, len(base) - 6)
    goal_fn, goal_mask, tree = _rejoin_goal(ctx, base, i_join_min)
    astar = HybridAStar(m2, car, dict(cfg["astar"], **rc.get("astar", {})), cl, recovery=rc)
    try:
        raw, n_exp = astar.plan([tuple(pose)], goal_fn, goal_mask, max_expand=rc["max_expand"],
                                check_start=False, h_weight=rc["h_weight"])
    except RuntimeError as e:
        return {"ok": False, "segments": [], "reason": str(e)}

    # tách thành các đoạn theo chiều chuyển động
    segments, i = [], 1
    while i < len(raw):
        d = int(raw[i, 4])
        j = i
        while j + 1 < len(raw) and int(raw[j + 1, 4]) == d:
            j += 1
        if d == 0:
            segments.append(("spin", float(raw[j, 2])))
        else:
            tr = _segment_traj(raw[i - 1:j + 1, :2], step, d < 0, car, m2, cl)
            segments.append(("fwd" if d > 0 else "rev", tr))
        i = j + 1

    d_end, jj = tree.query(raw[-1, :2])
    if d_end <= 30:  # nhập lại quỹ đạo base
        tail = base[i_join_min + int(jj):]
        if segments and segments[-1][0] == "fwd":
            segments[-1] = ("fwd", concat_traj(segments[-1][1], tail))
        else:
            segments.append(("fwd", concat_traj(np.zeros((0, 5)), tail)))
    return {"ok": True, "segments": segments, "raw": raw, "expanded": n_exp,
            "n_rev": sum(1 for s in segments if s[0] == "rev"), "n_spin": sum(1 for s in segments if s[0] == "spin")}


def plan_detour(ctx, nom, box, min_split_idx=0, inflate=0.0):
    """Nhánh né box (x, y): tách khỏi quỹ đạo chính trước box, nhập lại sau box.

    inflate: phóng to box (mm mỗi phía) theo độ bất định của vị trí ước lượng.
    Trả về dict: need (có cần né không), ok, traj (quỹ đạo đã ghép), i_split, i_join, clear_R.
    """
    cfg, car, cl, det = ctx.cfg, ctx.car, ctx.cl, ctx.cfg["detect"]
    rbox = [("R", float(box[0]), float(box[1]))]
    size = ctx.m.box + 2 * inflate
    dR = car.clearance_to(nom[:, 1:4], rbox, size)
    need_idx = np.nonzero(dR < cl["box_soft_mm"])[0]
    if len(need_idx) == 0:
        return {"need": False, "ok": True, "traj": nom, "clear_R": float(dR.min()),
                "i_split": None, "i_join": None}

    m2 = ctx.m.with_extra_boxes(rbox, inflate)
    step = cfg["output"]["sample_mm"]
    i0, i1 = int(need_idx[0]), int(need_idx[-1])
    astar = HybridAStar(m2, car, cfg["astar"], cl)
    sm = Smoother(m2, car, cfg["smooth"], cl, cfg["astar"]["r_min_mm"])

    for extra in (0, 250, 500, 800, -150):
        back = det["split_back_mm"] + extra
        i_split = max(int(min_split_idx), i0 - int(back / step), 4)
        i_join_min = min(len(nom) - 6, i1 + int(det["join_ahead_mm"] / step))
        if i_split >= i_join_min or len(nom[i_join_min:len(nom) - 5]) == 0:
            continue
        goal_fn, goal_mask, tree = _rejoin_goal(ctx, nom, i_join_min)

        try:
            raw, _ = astar.plan([tuple(nom[i_split, 1:4])], goal_fn, goal_mask,
                                max_expand=det["max_expand"], h_weight=det["h_weight"])
        except RuntimeError:
            continue
        d_end, j = tree.query(raw[-1, :2])
        rejoin = d_end <= 30
        i_join = i_join_min + int(j) if rejoin else len(nom) - 1
        a, b = i_split - 4, min(i_join + 4, len(nom) - 1)
        seg_in = np.vstack([nom[a, 1:3], raw[:, :2], nom[b, 1:3]]) if rejoin else \
            np.vstack([nom[a, 1:3], raw[:, :2]])
        seg = spline_sample(sm.run(seg_in), step)
        tail = nom[b + 1:] if rejoin else nom[:0]
        xy = np.vstack([nom[:a, 1:3], seg[:, 1:3], tail[:, 1:3]])
        th = np.concatenate([nom[:a, 3], seg[:, 3], tail[:, 3]])
        k = np.concatenate([nom[:a, 4], seg[:, 4], tail[:, 4]])
        traj = np.column_stack([_recompute_geometry(xy), xy, th, k])
        db, dw = car.clearance(m2, traj[:, 1:4])
        ok = db.min() >= cl["box_hard_mm"] and dw.min() >= cl["wall_hard_mm"]
        if not ok:
            continue
        return {"need": True, "ok": True, "traj": traj, "i_split": a, "i_join": a + len(seg),
                "rejoin": bool(rejoin),
                "clear_R": float(car.clearance_to(traj[:, 1:4], rbox, size).min()),
                "clear_box": float(db.min()), "clear_wall": float(dw.min())}
    return {"need": True, "ok": False, "traj": nom, "i_split": None, "i_join": None,
            "clear_R": float(dR.min())}


def speed_for(ctx, traj, v_max=None):
    sc = dict(ctx.cfg["speed"])
    if v_max is not None:
        sc["v_max_mps"] = v_max
    return speed_profile(traj, sc)
