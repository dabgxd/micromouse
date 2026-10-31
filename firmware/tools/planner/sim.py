"""Mô phỏng một lượt chạy: vật lý xe + cảm biến có nhiễu + thuật toán trên xe + chấm điểm theo luật."""
import math

import numpy as np
from matplotlib.path import Path

from estimator import Detector, Ekf, wrap
from partitions import box_partition, build_lines, crossing_index
from pipeline import speed_for
from sensors import Tof


class SimResult:
    pass


def _replan(ctx, detour_fn, nominal, ref, v_ref, est, idx, state, v_max, events, t, inflate=0.0):
    """Chọn quỹ đạo khi biết (hoặc cập nhật) tâm box. Trả về (ref, v_ref, state)."""
    def use(res, new_state, msg):
        events.append((t, msg))
        return res["traj"], speed_for(ctx, res["traj"], v_max)[0], new_state

    if ref is nominal:
        res = detour_fn(est, 0, nominal, inflate)
        if not res["need"]:
            return ref, v_ref, "không cần né"
        if res["ok"] and res["i_split"] is not None and res["i_split"] > idx + 2:
            return use(res, "nhánh né", "Chuyển sang nhánh né lập sẵn (phát hiện đủ sớm)")
    # phát hiện muộn hoặc tâm box thay đổi: lập nhánh từ vị trí hiện tại trên quỹ đạo đang bám
    res = detour_fn(est, idx + 8, ref, inflate)
    if not res["need"]:
        return ref, v_ref, state if state != "none" else "không cần né"
    if res["ok"]:
        return use(res, "né muộn", "Lập nhánh né từ vị trí hiện tại")
    events.append((t, "!! Không tìm được nhánh né"))
    return ref, v_ref, "không né được"


MODE_NAMES = ["tiến", "lùi", "xoay tại chỗ", "dừng", "đã dừng (hết quỹ đạo)"]


def _mode_code(mode, ref_dir, stopped_front, at_end):
    """Trạng thái chuyển động để ghi log (chỉ số trong MODE_NAMES)."""
    if mode == "halt" or (stopped_front and ref_dir > 0):
        return 3
    if mode == "spin":
        return 2
    if at_end:
        return 4
    return 1 if ref_dir < 0 else 0


def _next_segment(ctx, queue, v_max, rcv):
    """Lấy đoạn tiếp theo của quỹ đạo cứu. Trả về (mode, ref, v_ref, ref_dir, spin_target)."""
    kind, data = queue.pop(0)
    if kind == "spin":
        return "spin", None, None, 1, data
    # đoạn tiến cuối cùng (đã nối phần còn lại của quỹ đạo) chạy tốc độ bình thường, các đoạn khác chạy chậm
    vm = v_max if (kind == "fwd" and not queue) else rcv["v_mps"]
    return "track", data, speed_for(ctx, data, vm)[0], (1 if kind == "fwd" else -1), None


def simulate(ctx, nominal, random_box=None, seed=0, noise=1.0, use_ekf=True, v_max=None,
             detour_fn=None, recovery_fn=None):
    """nominal: quỹ đạo chính (N,5). random_box: (x, y) thật hoặc None.

    detour_fn(box_xy, min_split_idx, base) -> dict của pipeline.plan_detour (chuyển nhánh khi phát hiện box).
    recovery_fn(box_xy | None, pose, base, idx) -> dict của pipeline.plan_recovery (lùi / xoay khi kẹt).
    """
    cfg, car, m = ctx.cfg, ctx.car, ctx.m
    sc, cc, ec = cfg["sim"], cfg["control"], cfg["ekf"]
    rc = cfg["robot"]
    rng = np.random.default_rng(seed)
    g = lambda s: rng.normal(0.0, s * noise) if s > 0 and noise > 0 else 0.0  # noqa: E731

    static_xy = [(x, y) for _, x, y in m.boxes]
    true_boxes = static_xy + ([tuple(random_box)] if random_box is not None else [])
    m_true = m.with_extra_boxes([("R", *random_box)]) if random_box is not None else m
    tof = Tof(rc, m.box)
    poly = Path(m.polygon)

    # --- tham số "thật" của xe (khác với giá trị danh định mà thuật toán biết) ---
    r_nom = rc["wheel_diameter_mm"] / 2
    b_nom = rc["track_mm"]
    rL = r_nom * (1 + g(sc["wheel_scale_sigma"]))
    rR = r_nom * (1 + g(sc["wheel_scale_sigma"]))
    gyro_bias = math.radians(g(sc["gyro_bias_dps"]))
    gyro_scale = 1 + g(sc["gyro_scale_sigma"])
    cpr = rc["encoder_cpr_wheel"]

    ref = lref = nominal
    v_ref, _ = speed_for(ctx, ref, v_max)
    x0, y0, th0 = ref[0, 1:4]
    tx = x0 + g(sc["start_err_mm"])
    ty = y0 + g(sc["start_err_mm"])
    tth = th0 + math.radians(g(sc["start_err_deg"]))

    ekf = Ekf((x0, y0, th0), ec, tof, static_xy, sc["start_err_mm"] * max(noise, 0.2),
              sc["start_err_deg"] * max(noise, 0.2))
    det = Detector(m, cfg["detect"])

    dt = sc["dt_phys_s"]
    n_ctrl = max(1, round(sc["dt_ctrl_s"] / dt))
    n_tof = max(1, round(1.0 / sc["tof_hz"] / dt))
    tau = sc["motor_tau_s"]
    vw_max = sc["v_wheel_max_mps"]

    wl = wr = 0.0      # tốc độ vành bánh thật (m/s)
    wl_c = wr_c = 0.0  # lệnh
    phiL = phiR = 0.0
    cntL = cntR = 0
    idx = 0
    v_cmd = 0.0
    stopped_front = False
    last_front = None
    planned_est = None  # tâm box đã dùng để lập nhánh né gần nhất
    front_close = 0  # số lần đo M0 liên tiếp dưới ngưỡng
    stop_since = None
    # quỹ đạo cứu (lùi / xoay)
    rcv = cfg["recovery"]
    if not rcv.get("enable", True):
        recovery_fn = None
    mode = "track"        # "track" (bám quỹ đạo) | "spin" (xoay tại chỗ) | "halt" (dừng để lập quỹ đạo cứu)
    ref_dir = 1           # +1 tiến, -1 lùi
    spin_target = None
    queue = []            # các đoạn còn lại của quỹ đạo cứu
    in_recovery = False
    rec_base = None       # quỹ đạo gốc (tiến) lúc bắt đầu cứu
    rerecover = False
    halt_until = 0.0
    attempts = 0          # số lần lập quỹ đạo cứu THẤT BẠI
    n_recoveries = 0      # tổng số lần đã lập quỹ đạo cứu thành công
    need_recovery = False  # vừa không tìm được nhánh né đi tiến -> cần quỹ đạo cứu
    recovery_paths = []
    stop_thr = cc["stop_front_mm"]
    events = []
    detour_state = "none"
    finished_t = None

    log = {k: [] for k in ("t", "x", "y", "th", "ex", "ey", "eth", "idx", "v", "w", "e_lat", "e_est",
                           "clr_box", "clr_wall", "trP", "mode")}
    tof_log = []   # (t, i, r)
    tof_pts = []   # (x, y, status) điểm rơi theo pose ước lượng
    collided = set()
    wheel_out = [False, False]
    wheel_out_events = 0

    n_steps = int(sc["t_max_s"] / dt)
    for k in range(n_steps):
        t = k * dt
        # ---------- vật lý ----------
        wl += (wl_c - wl) * dt / tau
        wr += (wr_c - wr) * dt / tau
        dphiL, dphiR = wl * 1000 * dt / r_nom, wr * 1000 * dt / r_nom  # rad bánh quay
        slip = sc["slip_sigma"] * math.sqrt(dt / 0.01)
        dL = dphiL * rL * (1 + g(slip))
        dR = dphiR * rR * (1 + g(slip))
        ds, dth = (dL + dR) / 2, (dR - dL) / b_nom
        tx += ds * math.cos(tth + dth / 2)
        ty += ds * math.sin(tth + dth / 2)
        tth = wrap(tth + dth)
        phiL += dphiL
        phiR += dphiR

        # ---------- cảm biến + dự đoán (1 kHz) ----------
        nL, nR = math.floor(phiL * cpr / (2 * math.pi)), math.floor(phiR * cpr / (2 * math.pi))
        dsL = (nL - cntL) * 2 * math.pi / cpr * r_nom
        dsR = (nR - cntR) * 2 * math.pi / cpr * r_nom
        cntL, cntR = nL, nR
        gyro = (dth / dt) * gyro_scale + gyro_bias + math.radians(g(sc["gyro_noise_dps"]))
        ekf.predict((dsL + dsR) / 2, gyro * dt, dt)

        # ---------- ToF ----------
        if k % n_tof == 0:
            for i in range(5):
                r = tof.true_range((tx, ty, tth), i, true_boxes)
                if r is not None:
                    r = r + g(sc["tof_sigma_rel"] * r + sc["tof_sigma_mm"])
                    if noise > 0 and rng.random() < sc["tof_outlier_p"] * noise:
                        r = rng.uniform(30, tof.max_r)
                tof_log.append((t, i, np.nan if r is None else r))
                if r is None:
                    continue
                if tof.names[i] == "M0":
                    last_front = r
                    front_close = front_close + 1 if r < stop_thr else 0
                status = "none"
                innov = None
                if use_ekf:
                    status, innov = ekf.update_tof(i, r, sc["tof_sigma_rel"], sc["tof_sigma_mm"])
                else:
                    pred = tof.predict(ekf.x, i, static_xy, ec["max_range_mm"],
                                       math.radians(ec["max_incidence_deg"]))
                    if pred is not None:
                        innov = r - pred[0]
                        status = "rej" if abs(innov) > 60 else "acc"
                sx, sy, sa = tof.sensor_pose(ekf.x, i)
                px, py = sx + r * math.cos(sa), sy + r * math.sin(sa)
                tof_pts.append((px, py, status))
                # vật lạ: số đo ngắn hơn hẳn so với map đã biết
                exp_r = tof.true_range(ekf.x, i, static_xy)  # khoảng cách mong đợi theo map đã biết
                unexpected = exp_r is None or r < exp_r - 50
                if unexpected and r < cfg["detect"]["max_range_mm"]:
                    ev = det.add(sx, sy, sa, r, tof.offsets, t)
                    if ev is not None:
                        kind, est = ev
                        if kind == "new":
                            events.append((t, f"Phát hiện box, tâm ước lượng ({est[0]:.0f}, {est[1]:.0f})"))
                        shift = math.inf if planned_est is None else math.hypot(est[0] - planned_est[0],
                                                                               est[1] - planned_est[1])
                        tracking_fwd = mode == "track" and ref_dir > 0 and not in_recovery
                        j_box = int(np.argmin((lref[:, 1] - est[0]) ** 2 + (lref[:, 2] - est[1]) ** 2))
                        # đang chạy quỹ đạo cứu mà tâm box dịch đáng kể -> dừng lại, lập lại quỹ đạo cứu
                        if (recovery_fn is not None and (in_recovery or mode == "spin")
                                and shift > cfg["detect"]["replan_shift_mm"]):
                            planned_est = est
                            rerecover = True
                            events.append((t, f"Cập nhật tâm box ({est[0]:.0f}, {est[1]:.0f}) khi đang cứu, "
                                              f"lệch {shift:.0f} mm -> lập lại quỹ đạo cứu"))
                        if (detour_fn is not None and tracking_fwd and shift > cfg["detect"]["replan_shift_mm"]
                                and j_box > idx):
                            if kind == "update":
                                events.append((t, f"Cập nhật tâm box ({est[0]:.0f}, {est[1]:.0f}), "
                                                  f"lệch {shift:.0f} mm -> lập lại"))
                            planned_est = est
                            ref, v_ref, detour_state = _replan(ctx, detour_fn, nominal, ref, v_ref, est, idx,
                                                               detour_state, v_max, events, t, det.uncertainty())
                            need_recovery = detour_state == "không né được"

        # ---------- điều khiển (200 Hz) ----------
        if k % n_ctrl == 0:
            ex, ey, eth = ekf.x
            dts = n_ctrl * dt
            still = abs(wl) < 0.005 and abs(wr) < 0.005
            v = w = 0.0
            at_end = False

            if rerecover and mode != "halt" and n_recoveries < rcv["max_recoveries"]:
                rerecover = False
                queue, in_recovery, mode = [], False, "halt"
            # --- kích hoạt quỹ đạo cứu: kẹt trước vật cản, hoặc không còn nhánh né đi tiến ---
            if (recovery_fn is not None and mode == "track" and ref_dir > 0 and not in_recovery
                    and attempts < rcv["max_attempts"] and n_recoveries < rcv["max_recoveries"]):
                stuck = stop_since is not None and t - stop_since > rcv["trigger_stop_s"]
                if stuck or need_recovery:
                    need_recovery = False
                    mode = "halt"
                    events.append((t, "Dừng lại để lập quỹ đạo cứu (cho phép lùi / xoay)"))

            if mode == "halt":
                v_cmd = 0.0
                if still and t >= halt_until:
                    base = rec_base if rec_base is not None else ref
                    i_base = int(np.argmin((base[:, 1] - ex) ** 2 + (base[:, 2] - ey) ** 2))
                    res_r = recovery_fn(det.est, (ex, ey, eth), base, i_base)
                    if res_r["ok"]:
                        n_recoveries += 1
                        queue = list(res_r["segments"])
                        recovery_paths.append(res_r["segments"])
                        events.append((t, f"Quỹ đạo cứu: {res_r['n_rev']} đoạn lùi, {res_r['n_spin']} lần xoay"))
                        in_recovery = True
                        rec_base = base  # quỹ đạo gốc để nhập lại nếu phải lập lại quỹ đạo cứu
                        stopped_front, front_close, stop_since = False, 0, None
                        detour_state = "cứu (lùi/xoay)"
                        mode, ref, v_ref, ref_dir, spin_target = _next_segment(ctx, queue, v_max, rcv)
                        idx = 0
                    else:
                        attempts += 1
                        events.append((t, f"!! Không lập được quỹ đạo cứu (lần {attempts})"))
                        if rec_base is not None and attempts < rcv["max_attempts"] + 2:
                            halt_until = t + 0.5  # đang giữa quỹ đạo cứu: đứng yên, thử lại sau
                        else:
                            mode = "track"

            elif mode == "spin":
                err = wrap(spin_target - eth)
                w_max = math.radians(rcv["spin_rate_dps"])
                w = max(-w_max, min(w_max, 4.0 * err))
                if abs(err) < math.radians(2.0) and abs(wr - wl) < 0.01:
                    mode, ref, v_ref, ref_dir, spin_target = _next_segment(ctx, queue, v_max, rcv)
                    idx, v_cmd = 0, 0.0

            else:  # bám quỹ đạo (tiến hoặc lùi)
                N = len(ref)
                j0, j1 = idx, min(N, idx + 40)
                d2 = (ref[j0:j1, 1] - ex) ** 2 + (ref[j0:j1, 2] - ey) ** 2
                idx = j0 + int(np.argmin(d2))
                seg_end = idx >= N - 2
                if seg_end and queue:
                    mode, ref, v_ref, ref_dir, spin_target = _next_segment(ctx, queue, v_max, rcv)
                    idx, v_cmd = 0, 0.0
                else:
                    if in_recovery and not queue and idx * cfg["output"]["sample_mm"] > 300:
                        in_recovery = False
                        events.append((t, "Đã nhập lại quỹ đạo, chạy tiếp"))
                    at_end = seg_end
                    _, xr, yr, thr, kr = ref[idx]
                    look = min(N - 1, idx + int(cc["lookahead_mm"] / cfg["output"]["sample_mm"]))
                    vr = 0.0 if at_end else max(v_ref[look], cc["v_min_mps"])
                    # phanh khẩn cấp (chỉ khi đi tiến): 2 lần đo M0 liên tiếp quá gần, tự chạy lại khi trống
                    if ref_dir > 0 and front_close >= 2 and not at_end:
                        if not stopped_front:
                            events.append((t, f"!! Phanh khẩn cấp: M0 = {last_front:.0f} mm"))
                        stopped_front = True
                    elif stopped_front and front_close == 0 and last_front is not None \
                            and last_front > stop_thr + 40:
                        stopped_front = False
                        events.append((t, "Phía trước đã trống, chạy tiếp"))
                    if stopped_front and ref_dir > 0:
                        vr = 0.0
                    v_cmd = min(vr, v_cmd + cfg["speed"]["a_acc_mps2"] * dts) if vr > v_cmd else vr
                    # khi lùi: điều khiển "xe ảo" quay ngược 180°, rồi đảo dấu vận tốc
                    veth = eth if ref_dir > 0 else wrap(eth + math.pi)
                    c, s = math.cos(veth), math.sin(veth)
                    dx, dy = (xr - ex) / 1000, (yr - ey) / 1000
                    xe, ye = c * dx + s * dy, -s * dx + c * dy
                    the = wrap(thr - veth)
                    if v_cmd > 0:
                        v = v_cmd * math.cos(the) + cc["k_x"] * xe
                        w = v_cmd * (kr * 1000 + cc["k_y"] * ye + cc["k_theta"] * math.sin(the))
                    if ref_dir < 0:
                        v = -v
            stop_thr = rcv["stop_front_mm"] if in_recovery else cc["stop_front_mm"]
            lref = ref if ref is not None else lref  # quỹ đạo gần nhất (khi xoay tại chỗ ref = None)
            b = b_nom / 1000
            wl_c, wr_c = v - w * b / 2, v + w * b / 2
            sat = max(abs(wl_c), abs(wr_c)) / vw_max
            if sat > 1:
                wl_c, wr_c = wl_c / sat, wr_c / sat

            # ghi log + chỉ số thật
            pose = np.array([[tx, ty, tth]])
            db, dw = car.clearance(m_true, pose)
            jt = int(np.argmin((lref[:, 1] - tx) ** 2 + (lref[:, 2] - ty) ** 2))
            nrm = np.array([-math.sin(lref[jt, 3]), math.cos(lref[jt, 3])])
            e_lat = float(np.dot([tx - lref[jt, 1], ty - lref[jt, 2]], nrm))
            for key, val in (("t", t), ("x", tx), ("y", ty), ("th", tth), ("ex", ex), ("ey", ey),
                             ("eth", eth), ("idx", idx), ("v", v), ("w", w), ("e_lat", e_lat),
                             ("e_est", math.hypot(ex - tx, ey - ty)), ("clr_box", db[0]),
                             ("clr_wall", dw[0]), ("trP", math.sqrt(ekf.P[0, 0] + ekf.P[1, 1])),
                             ("mode", _mode_code(mode, ref_dir, stopped_front, at_end))):
                log[key].append(val)
            if db[0] < 0:
                for bid, bx, by in m_true.boxes:
                    if car.clearance_to(pose, [(bid, bx, by)], m.box)[0] < 0 and bid not in collided:
                        collided.add(bid)
                        events.append((t, f"!! Va chạm box #{bid}"))
            for wi, wc in enumerate(car.wheels):
                wx, wy = car.world(pose, wc)
                out = not poly.contains_points(np.column_stack([wx[0], wy[0]])).any()
                if out and not wheel_out[wi]:
                    wheel_out_events += 1
                    events.append((t, f"!! Bánh {'trái' if wi == 0 else 'phải'} ra ngoài vạch"))
                wheel_out[wi] = out

            # về đích: cả 4 góc xe qua vạch đích
            if finished_t is None:
                fl = m.finish_line
                wx, wy = car.world(pose, car.corners)
                cs = np.column_stack([wx[0], wy[0]])
                if np.all(cs[:, 1] < fl["y"]) and np.all((cs[:, 0] > fl["x_min"]) & (cs[:, 0] < fl["x_max"])):
                    finished_t = t
                    events.append((t, "Về đích"))
            if stopped_front:
                stop_since = t if stop_since is None else stop_since
            else:
                stop_since = None
            still = still and t > 0.5
            gave_up = (stop_since is not None and t - stop_since > 3.0
                       and (recovery_fn is None or attempts >= rcv["max_attempts"]
                            or n_recoveries >= rcv["max_recoveries"]))
            if still and (at_end or gave_up):
                if not at_end:
                    events.append((t, "!! Kẹt quá 3 s trước vật cản, dừng lượt"))
                break

    res = SimResult()
    res.log = {k: np.array(v) for k, v in log.items()}
    res.tof = np.array(tof_log) if tof_log else np.zeros((0, 3))
    res.tof_pts = tof_pts
    res.events = events
    res.ref = lref
    res.detected = [] if det.est is None else [det.est]
    res.det_history = det.history
    res.detour_state = detour_state
    res.recovery_paths = recovery_paths
    res.finished_t = finished_t
    res.collided = collided
    res.wheel_out_events = wheel_out_events
    res.random_box = random_box
    res.score = score(ctx, res)
    return res


def score(ctx, res):
    """Chấm điểm lượt 2 theo luật (Điều 3)."""
    m, car = ctx.m, ctx.car
    L = res.log
    lines = build_lines(m)
    true_traj = np.column_stack([np.zeros(len(L["x"])), L["x"], L["y"], L["th"]])
    cross = crossing_index(car, true_traj, lines)
    passed = {ln.k for ln, c in zip(lines, cross) if c is not None}
    pts, detail = 0, []
    n_static = n_rand = 0
    for bid, x, y in m.boxes:
        k = box_partition(m, lines, x, y)
        if k in passed and bid not in res.collided:
            n_static += 1
    if res.random_box is not None:
        k = box_partition(m, lines, *res.random_box)
        if k in passed and "R" not in res.collided:
            n_rand = 1
    pts += 10 * n_static + 15 * n_rand
    detail.append(f"Vượt box: {n_static}×10 + {n_rand}×15")
    if res.finished_t is not None:
        pts += 40
        sp = max(0.0, (240 - res.finished_t) / 6)
        pts += sp
        detail.append(f"Về đích +40, tốc độ +{sp:.1f} ({res.finished_t:.2f} s)")
    pen = 5 * len(res.collided) + 5 * res.wheel_out_events
    if pen:
        detail.append(f"Trừ: xô box {len(res.collided)}×5, bánh ra ngoài {res.wheel_out_events}×5")
    if res.finished_t is not None and pen == 0:
        pts += 10
        detail.append("Chạy sạch +10")
    pts = max(0.0, pts - pen)
    return {"total": pts, "detail": detail, "n_static": n_static, "n_rand": n_rand}


def summary(res):
    L = res.log
    return {
        "finished": res.finished_t is not None,
        "t_finish": res.finished_t,
        "score": res.score["total"],
        "max_e_lat_mm": float(np.abs(L["e_lat"]).max()) if len(L["e_lat"]) else float("nan"),
        "max_e_est_mm": float(L["e_est"].max()) if len(L["e_est"]) else float("nan"),
        "min_clr_box_mm": float(L["clr_box"].min()) if len(L["clr_box"]) else float("nan"),
        "min_clr_wall_mm": float(L["clr_wall"].min()) if len(L["clr_wall"]) else float("nan"),
        "collisions": len(res.collided),
        "wheel_out": res.wheel_out_events,
        "detected": res.detected,
        "box_est_err_mm": (float(math.hypot(res.detected[0][0] - res.random_box[0],
                                            res.detected[0][1] - res.random_box[1]))
                           if res.detected and res.random_box is not None else None),
        "detour": res.detour_state,
    }
