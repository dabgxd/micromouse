"""Công cụ tương tác: đặt box ngẫu nhiên ở ô 4/5, lập nhánh né, mô phỏng lượt chạy và chấm điểm.

Chạy (từ thư mục gốc project):
    tools/.venv/bin/python tools/planner/sim_gui.py              # giao diện
    tools/.venv/bin/python tools/planner/sim_gui.py --box 1250 350 --headless
    tools/.venv/bin/python tools/planner/sim_gui.py --box 1250 350 --mc 30
    tools/.venv/bin/python tools/planner/sim_gui.py --sweep      # quét vị trí box trên lưới 50 mm
    tools/.venv/bin/python tools/planner/sim_gui.py --random 100 # quét 100 vị trí box ngẫu nhiên
    tools/.venv/bin/python tools/planner/sim_gui.py --box 1304 674 --animate run.gif   # xuất hoạt hình

Trong giao diện: click trái vào ô 4/5 để đặt box (vị trí tự do, không theo lưới), click phải để xoá box.
"▶ Phát" để xem lại xe chạy theo quỹ đạo; kéo thanh "t (s)" để tua.
"""
import argparse
import csv
import os
import sys
import textwrap
import time


def parse_args():
    p = argparse.ArgumentParser(description="Mô phỏng xe dò mê cung")
    p.add_argument("--box", nargs=2, type=float, metavar=("X", "Y"), help="toạ độ tâm box ngẫu nhiên (mm)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--noise", type=float, default=1.0, help="hệ số nhân nhiễu (0 = không nhiễu)")
    p.add_argument("--vmax", type=float, default=None, help="vận tốc tối đa (m/s)")
    p.add_argument("--no-ekf", action="store_true", help="tắt EKF (chỉ odometry + gyro)")
    p.add_argument("--no-recovery", action="store_true", help="không cho phép lùi / xoay tại chỗ khi kẹt")
    p.add_argument("--headless", action="store_true", help="không mở cửa sổ, lưu PNG/CSV vào out/sim")
    p.add_argument("--mc", type=int, default=0, help="chạy Monte-Carlo N lần (headless)")
    p.add_argument("--sweep", action="store_true", help="quét vị trí box trên lưới --step mm (headless)")
    p.add_argument("--step", type=float, default=50.0, help="bước lưới khi --sweep (mm)")
    p.add_argument("--random", type=int, default=0, help="quét N vị trí box ngẫu nhiên liên tục (headless)")
    p.add_argument("--animate", metavar="FILE.gif", help="xuất hoạt hình lượt chạy ra file GIF (headless)")
    p.add_argument("--anim-speed", type=float, default=1.0, help="tốc độ phát của GIF (1 = thời gian thực)")
    return p.parse_args()


ARGS = parse_args() if __name__ == "__main__" else None

import matplotlib  # noqa: E402

if ARGS is None or ARGS.headless or ARGS.mc or ARGS.sweep or ARGS.random or ARGS.animate:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.widgets import Button, CheckButtons, Slider  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from partitions import build_lines, car_corners  # noqa: E402
from pipeline import OUT, Context, nominal_cached, plan_detour, plan_nominal, plan_recovery  # noqa: E402
from sensors import Tof  # noqa: E402
from sim import MODE_NAMES, simulate, summary  # noqa: E402

SIM_OUT = os.path.join(OUT, "sim")
TOF_COLORS = ["#1f77b4", "#2ca02c", "#d62728", "#9467bd", "#ff7f0e"]
FRAME_S = 0.04  # chu kỳ khung hình khi phát lại trong giao diện (s)


def draw_base(ax, m):
    ax.set_aspect("equal")
    ax.add_patch(Polygon(m.polygon, closed=True, fc="#f3f3ee", ec="black", lw=2, zorder=0))
    ax.add_patch(Rectangle((600, 0), 600, 600, fc="#f6d5d0", ec="none", hatch="//", zorder=0))
    for t in m.tiles:
        rnd = t["id"] in m.random_tiles
        ax.add_patch(Rectangle((t["x0"], t["y0"]), m.tile, m.tile, fc="#fff6d6" if rnd else "none",
                               ec="#bbb", lw=0.8, zorder=0))
        ax.text(t["x0"] + 12, t["y0"] + m.tile - 40, f"Ô {t['id']}", color="#999", fontsize=9)
    for ln in build_lines(m):
        ax.plot(*np.array([ln.a, ln.b]).T, color="#b89be0", lw=0.6, ls="--", zorder=1)
    for bid, x, y in m.boxes:
        ax.add_patch(Rectangle((x - 50, y - 50), 100, 100, fc="#ccc", ec="#444", lw=1, zorder=2))
        ax.text(x, y, str(bid), ha="center", va="center", fontsize=7, zorder=3)
    ax.plot([0, 600], [0, 0], color="green", lw=3)
    ax.plot([1200, 1800], [0, 0], color="black", lw=3, ls=(0, (2, 2)))
    ax.set_xlim(-40, 1840)
    ax.set_ylim(-320, 1240)
    ax.tick_params(labelsize=7)


class App:
    def __init__(self, ctx, nom, interactive=True):
        self.ctx, self.nom = ctx, nom
        self.interactive = interactive
        cfg = ctx.cfg
        self.box = None
        self.v_max = cfg["speed"]["v_max_mps"]
        self.noise = 1.0
        self.seed = 0
        self.use_ekf = cfg["ekf"]["enable"]
        self.use_recovery = cfg["recovery"]["enable"]
        self.show_global = False
        self.detour = self.glob = self.res = self.mc = None
        self._detour_cache, self._global_cache = {}, {}
        self.dyn = []
        self.anim = None       # các artist của hoạt hình (xe, tia ToF, vệt đường, ...)
        self.cursors = []      # con trỏ thời gian trên các biểu đồ
        self.playing = False
        self.play_speed = 1.0
        self.tof = Tof(cfg["robot"], ctx.m.box)

        self.fig = plt.figure(figsize=(17, 9.6))
        try:
            self.fig.canvas.manager.set_window_title("Mô phỏng xe dò mê cung")
        except Exception:
            pass
        self.ax = self.fig.add_axes([0.03, 0.235, 0.5, 0.715])
        self.axp = [self.fig.add_axes([0.59, y, 0.395, 0.15]) for y in (0.81, 0.615, 0.42, 0.225)]
        self.ax_info = self.fig.add_axes([0.585, 0.0, 0.41, 0.19])
        self.ax_info.axis("off")
        self.status = self.fig.text(0.03, 0.965, "", fontsize=11)
        draw_base(self.ax, ctx.m)
        if interactive:
            self._widgets()
            self.fig.canvas.mpl_connect("button_press_event", self.on_click)

    # ---------- giao diện ----------
    def _widgets(self):
        f = self.fig
        self.b_run = Button(f.add_axes([0.03, 0.125, 0.1, 0.045]), "▶ Mô phỏng")
        self.b_mc = Button(f.add_axes([0.14, 0.125, 0.12, 0.045]), "Monte-Carlo ×20")
        self.b_clr = Button(f.add_axes([0.27, 0.125, 0.09, 0.045]), "Xoá box")
        self.b_save = Button(f.add_axes([0.37, 0.125, 0.1, 0.045]), "Lưu kết quả")
        self.b_run.on_clicked(lambda _: self.run())
        self.b_mc.on_clicked(lambda _: self.run_mc(20))
        self.b_clr.on_clicked(lambda _: self.set_box(None))
        self.b_save.on_clicked(lambda _: self.save())
        self.b_gif = Button(f.add_axes([0.48, 0.125, 0.05, 0.045]), "GIF")
        self.b_gif.on_clicked(lambda _: self.save_animation())
        # phát lại
        self.b_play = Button(f.add_axes([0.03, 0.18, 0.06, 0.035]), "▶ Phát")
        self.b_play.on_clicked(lambda _: self.toggle_play())
        self.s_t = Slider(f.add_axes([0.125, 0.187, 0.36, 0.02]), "t (s)", 0.0, 1.0, valinit=0.0)
        self.s_t.on_changed(self.draw_frame)
        self.timer = f.canvas.new_timer(interval=int(FRAME_S * 1000))
        self.timer.add_callback(self._tick)
        self.chk = CheckButtons(f.add_axes([0.03, 0.005, 0.16, 0.11]),
                                ["EKF (dùng ToF)", "So sánh QĐ toàn cục", "Cho phép lùi / xoay khi kẹt"],
                                [self.use_ekf, self.show_global, self.use_recovery])
        self.chk.on_clicked(self._on_check)
        self.s_v = Slider(f.add_axes([0.28, 0.095, 0.2, 0.02]), "v_max (m/s)", 0.15, 0.9,
                          valinit=self.v_max, valstep=0.05)
        self.s_n = Slider(f.add_axes([0.28, 0.068, 0.2, 0.02]), "Nhiễu ×", 0.0, 3.0,
                          valinit=self.noise, valstep=0.1)
        self.s_s = Slider(f.add_axes([0.28, 0.041, 0.2, 0.02]), "Seed", 0, 99, valinit=0, valstep=1)
        self.s_p = Slider(f.add_axes([0.28, 0.014, 0.2, 0.02]), "Tốc độ phát ×", 0.25, 4.0,
                          valinit=self.play_speed, valstep=0.25)
        self.s_p.on_changed(lambda v: setattr(self, "play_speed", float(v)))
        self.s_v.on_changed(lambda v: setattr(self, "v_max", float(v)))
        self.s_n.on_changed(lambda v: setattr(self, "noise", float(v)))
        self.s_s.on_changed(lambda v: setattr(self, "seed", int(v)))

    def _on_check(self, label):
        st = self.chk.get_status()
        self.use_ekf, self.show_global, self.use_recovery = st
        if label.startswith("So sánh") and self.box is not None:
            self.run()

    def set_status(self, msg, err=False):
        self.status.set_text(msg)
        self.status.set_color("#c0392b" if err else "#222")
        self._flush()

    def _flush(self):
        if self.interactive:
            self.fig.canvas.draw_idle()
            plt.pause(0.001)

    def on_click(self, ev):
        if ev.inaxes is not self.ax or ev.xdata is None:
            return
        tb = getattr(self.fig.canvas, "toolbar", None)
        if tb is not None and getattr(tb, "mode", ""):
            return  # đang zoom / pan
        if ev.button == 3:
            self.set_box(None)
            return
        if ev.button != 1:
            return
        x, y = ev.xdata, ev.ydata
        if not self.ctx.m.box_fits(x, y):
            self.set_status(f"({x:.0f}, {y:.0f}) không hợp lệ: box phải nằm gọn trong ô 4/5 và không chồng box tĩnh",
                            err=True)
            return
        self.set_box((x, y))

    def set_box(self, box):
        self.box = box
        self.run()

    # ---------- tính toán ----------
    def detour_fn(self, box, min_idx, base=None, inflate=0.0):
        """Nhánh né quanh box (x, y) liên tục. Chỉ cache khi lập từ quỹ đạo chính."""
        if base is not None and base is not self.nom:
            return plan_detour(self.ctx, base, box, min_idx, inflate=inflate)
        key = (round(box[0]), round(box[1]), int(min_idx), round(inflate))
        if key not in self._detour_cache:
            self._detour_cache[key] = plan_detour(self.ctx, self.nom, box, min_idx, inflate=inflate)
        return self._detour_cache[key]

    def recovery_fn(self, box, pose, base, idx):
        return plan_recovery(self.ctx, base, box, pose, idx)

    def global_plan(self, box):
        key = (round(box[0]), round(box[1]))
        if key not in self._global_cache:
            m2 = self.ctx.m.with_extra_boxes([("R", *box)])
            try:
                self._global_cache[key] = plan_nominal(self.ctx, m=m2, log=lambda *a: None)[0]
            except RuntimeError:
                self._global_cache[key] = None
        return self._global_cache[key]

    def _plan(self):
        self.detour = self.glob = None
        if self.box is not None:
            self.set_status("Đang lập nhánh né (Hybrid A*) ...")
            self.detour = self.detour_fn(self.box, 0)
            if self.show_global:
                self.set_status("Đang lập quỹ đạo toàn cục (biết trước box) ...")
                self.glob = self.global_plan(self.box)

    def _sim(self, seed):
        return simulate(self.ctx, self.nom, self.box, seed=seed, noise=self.noise, use_ekf=self.use_ekf,
                        v_max=self.v_max, detour_fn=self.detour_fn,
                        recovery_fn=self.recovery_fn if self.use_recovery else None)

    def run(self):
        t0 = time.time()
        self.stop_play()
        self.mc = None
        self._plan()
        self.set_status("Đang mô phỏng ...")
        self.res = self._sim(self.seed)
        self.redraw()
        self.set_status(f"Xong ({time.time() - t0:.1f} s).  Click trái vào ô 4/5: đặt box  |  click phải: xoá box")

    def run_mc(self, n):
        t0 = time.time()
        self.stop_play()
        self._plan()
        runs = []
        for i in range(n):
            self.set_status(f"Monte-Carlo {i + 1}/{n} ...")
            runs.append(self._sim(self.seed + i))
        self.mc = runs
        self.res = runs[0]
        self.redraw()
        self.set_status(f"Monte-Carlo {n} lần xong ({time.time() - t0:.1f} s)")
        return runs

    # ---------- vẽ ----------
    def _clear_dyn(self):
        for a in self.dyn:
            try:
                a.remove()
            except Exception:
                pass
        self.dyn = []

    def redraw(self):
        self._clear_dyn()
        ax, d, car = self.ax, self.dyn, self.ctx.car
        d += ax.plot(self.nom[:, 1], self.nom[:, 2], color="#777", lw=1.2, label="Quỹ đạo chính")
        if self.detour is not None and self.detour["need"]:
            if self.detour["ok"]:
                tr = self.detour["traj"]
                d += ax.plot(tr[:, 1], tr[:, 2], color="#f39c12", lw=1.6, ls="--", label="Nhánh né (lập sẵn)")
        if self.glob is not None:
            d += ax.plot(self.glob[:, 1], self.glob[:, 2], color="#8e44ad", lw=1.4, ls=":",
                         label="QĐ toàn cục (biết trước box)")
        if self.box is not None:
            d.append(ax.add_patch(Rectangle((self.box[0] - 50, self.box[1] - 50), 100, 100,
                                            fc="#ffc845", ec="#a05a00", lw=1.5, zorder=4)))
            d.append(ax.text(self.box[0], self.box[1], "R", ha="center", va="center", fontsize=8, zorder=5))

        if self.mc:
            for i, r in enumerate(self.mc):
                L = r.log
                bad = r.finished_t is None or r.collided or r.wheel_out_events
                d += ax.plot(L["x"], L["y"], color="#c0392b" if bad else "#2e86de", lw=0.7, alpha=0.5,
                             label="Đường thật (Monte-Carlo)" if i == 0 else None)
        elif self.res is not None:
            r, L = self.res, self.res.log
            for k in range(0, len(L["x"]), 60):
                d.append(ax.add_patch(Polygon(car_corners(car, (L["x"][k], L["y"][k], L["th"][k])), closed=True,
                                              fc="none", ec="#2e86de", lw=0.4, alpha=0.3, zorder=3)))
            d += ax.plot(L["x"], L["y"], color="#2e86de", lw=1, alpha=0.4, zorder=6)
            d += ax.plot(L["ex"], L["ey"], color="#e74c3c", lw=1, ls="--", label="Vị trí ước lượng (EKF)", zorder=6)
            pts = np.array([(p[0], p[1]) for p in r.tof_pts]) if r.tof_pts else np.zeros((0, 2))
            stt = np.array([p[2] for p in r.tof_pts])
            for s, col, size, lab in (("none", "#999", 2, "ToF: không dùng"), ("acc", "#27ae60", 7, "ToF: EKF nhận"),
                                      ("rej", "#e67e22", 9, "ToF: EKF loại")):
                sel = stt == s if len(stt) else np.zeros(0, bool)
                if sel.any():
                    d.append(ax.scatter(pts[sel, 0], pts[sel, 1], s=size, color=col, alpha=0.6, label=lab, zorder=5))
            labeled = set()
            for n, segs in enumerate(r.recovery_paths):
                for kind, data in segs:
                    if kind == "spin":
                        continue
                    lab = {"rev": "Quỹ đạo cứu: LÙI", "fwd": "Quỹ đạo cứu: tiến"}[kind]
                    seg = data if kind == "rev" else data[:min(len(data), 60)]
                    d += ax.plot(seg[:, 1], seg[:, 2], color="#d6249f" if kind == "rev" else "#16a085", lw=2.2,
                                 ls="--", zorder=8, label=None if lab in labeled else lab)
                    labeled.add(lab)
                spins = [i for i, (kind, _) in enumerate(segs) if kind == "spin"]
                for i in spins:
                    prev = next((segs[j][1] for j in range(i - 1, -1, -1) if segs[j][0] != "spin"), None)
                    nxt = next((segs[j][1] for j in range(i + 1, len(segs)) if segs[j][0] != "spin"), None)
                    p = prev[-1, 1:3] if prev is not None else (nxt[0, 1:3] if nxt is not None else None)
                    if p is not None:
                        d += ax.plot(*p, "o", mfc="none", mec="#d6249f", ms=12, mew=2, zorder=9,
                                     label="Xoay tại chỗ" if (n == 0 and i == spins[0]) else None)
            if r.det_history:
                hx = [h[1] for h in r.det_history]
                hy = [h[2] for h in r.det_history]
                d += ax.plot(hx, hy, ".-", color="#c0392b", ms=3, lw=0.6, zorder=7,
                             label="Tâm box ước lượng (theo thời gian)")
            for cx, cy in r.detected:
                d.append(ax.add_patch(Rectangle((cx - 50, cy - 50), 100, 100, fc="none", ec="#c0392b", lw=1.5,
                                                ls="--", zorder=6)))
            for te, msg in r.events:
                if msg.startswith("!!") and len(L["t"]):
                    j = min(np.searchsorted(L["t"], te), len(L["t"]) - 1)
                    d += ax.plot(L["x"][j], L["y"][j], "x", color="#c0392b", ms=10, mew=2, zorder=7)
        self._make_anim_artists()
        leg = ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
        d.append(leg)
        self._draw_plots()
        self._draw_info()
        if self.anim is not None:
            T = float(self.res.log["t"][-1]) if len(self.res.log["t"]) else 1.0
            if self.interactive:
                self.s_t.valmax = T
                self.s_t.ax.set_xlim(0, T)
                self.s_t.set_val(T)  # mặc định hiện trạng thái cuối
            else:
                self.draw_frame(T)
        self._flush()

    def _draw_plots(self):
        self.cursors = []
        for a in self.axp:
            a.clear()
            a.grid(alpha=0.3)
            a.tick_params(labelsize=7)
        a1, a2, a3, a4 = self.axp
        hard = self.ctx.cl["box_hard_mm"]
        if self.mc:
            for r in self.mc:
                a1.plot(r.log["t"], r.log["e_lat"], lw=0.6, alpha=0.6)
                a2.plot(r.log["t"], r.log["clr_box"], lw=0.6, alpha=0.6)
            ts = [r.finished_t for r in self.mc if r.finished_t is not None]
            sc = [r.score["total"] for r in self.mc]
            if ts:
                a3.hist(ts, bins=15, color="#2e86de")
            a3.set_title("Phân bố thời gian về đích (s)", fontsize=8)
            a4.hist(sc, bins=15, color="#27ae60")
            a4.set_title("Phân bố điểm", fontsize=8)
            a1.set_title("Lệch ngang thật so với quỹ đạo (mm) - mọi lần chạy", fontsize=8)
            a2.set_title("Khoảng cách thân xe tới box gần nhất (mm)", fontsize=8)
            a2.axhline(0, color="red", lw=1)
            return
        if self.res is None:
            return
        L = self.res.log
        t = L["t"]
        a1.plot(t, L["e_lat"], label="lệch ngang thật vs quỹ đạo")
        a1.plot(t, L["e_est"], label="|ước lượng − thật|")
        a1.plot(t, L["trP"], ":", label="σ vị trí của EKF")
        a1.set_title("Sai số (mm)", fontsize=8)
        a1.legend(fontsize=6, loc="upper left")
        a2.plot(t, L["clr_box"], label="tới box")
        a2.plot(t, L["clr_wall"], label="tới biên")
        a2.axhline(hard, color="red", ls=":", lw=1)
        a2.axhline(0, color="black", lw=0.8)
        a2.set_ylim(-20, 200)
        a2.set_title("Khoảng cách thân xe (mm)", fontsize=8)
        a2.legend(fontsize=6, loc="upper left")
        a3.plot(t, L["v"], label="v (m/s)")
        a3.plot(t, np.array(L["w"]) / 10, label="ω/10 (rad/s)")
        a3.set_title("Lệnh vận tốc", fontsize=8)
        a3.legend(fontsize=6, loc="upper left")
        tof = self.res.tof
        for i in range(5):
            sel = tof[:, 1] == i
            a4.plot(tof[sel, 0], tof[sel, 2], ".", ms=2, color=TOF_COLORS[i],
                    label=self.ctx.cfg["robot"]["tof"][i]["name"])
        a4.set_title("ToF đo được (mm)", fontsize=8)
        a4.set_xlabel("t (s)", fontsize=7)
        a4.legend(fontsize=6, loc="upper left", ncol=5, markerscale=4)
        self.cursors = [a.axvline(t[-1] if len(t) else 0, color="black", lw=0.9, alpha=0.7) for a in self.axp]

    # ---------- phát lại (hoạt hình) ----------
    def _make_anim_artists(self):
        """Tạo các artist động cho lượt chạy hiện tại (chỉ khi xem một lượt, không phải Monte-Carlo)."""
        self.anim = None
        if self.mc or self.res is None or not len(self.res.log["t"]):
            return
        ax, d = self.ax, self.dyn
        a = {
            "trail": ax.plot([], [], color="#2e86de", lw=2.5, zorder=11, label="Đường đi thật")[0],
            "est_trail": ax.plot([], [], color="#e74c3c", lw=1, ls="--", zorder=11,
                                 label="Vị trí ước lượng (EKF)")[0],
            "car": ax.add_patch(Polygon(np.zeros((4, 2)), closed=True, fc="#2e86de", ec="#1b4f8a", lw=1.2,
                                        alpha=0.6, zorder=12)),
            "est": ax.add_patch(Polygon(np.zeros((4, 2)), closed=True, fc="none", ec="#e74c3c", lw=1.2,
                                        ls="--", zorder=12)),
            "nose": ax.plot([], [], color="#1b4f8a", lw=2, zorder=13)[0],
            "rays": [ax.plot([], [], color=c, lw=1.4, zorder=12)[0] for c in TOF_COLORS],
            "hits": [ax.plot([], [], "o", color=c, ms=4, zorder=13)[0] for c in TOF_COLORS],
            "text": ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left", fontsize=8.5,
                            family="monospace", zorder=14,
                            bbox=dict(boxstyle="round", fc="white", ec="#888", alpha=0.9)),
        }
        d += [a["trail"], a["est_trail"], a["car"], a["est"], a["nose"], a["text"]] + a["rays"] + a["hits"]
        # số đo ToF theo từng cảm biến, sắp theo thời gian
        tof = self.res.tof
        a["tof"] = [(tof[tof[:, 1] == i, 0], tof[tof[:, 1] == i, 2]) for i in range(5)]
        a["ev_t"] = np.array([e[0] for e in self.res.events])
        self.anim = a

    def draw_frame(self, t):
        """Vẽ trạng thái lượt chạy tại thời điểm t (s)."""
        a = self.anim
        if a is None:
            return
        L, car = self.res.log, self.ctx.car
        k = int(min(np.searchsorted(L["t"], t, side="right"), len(L["t"])) - 1)
        k = max(k, 0)
        pose = (L["x"][k], L["y"][k], L["th"][k])
        a["trail"].set_data(L["x"][:k + 1], L["y"][:k + 1])
        a["est_trail"].set_data(L["ex"][:k + 1], L["ey"][:k + 1])
        a["car"].set_xy(car_corners(car, pose))
        a["est"].set_xy(car_corners(car, (L["ex"][k], L["ey"][k], L["eth"][k])))
        c, s = np.cos(pose[2]), np.sin(pose[2])
        a["nose"].set_data([pose[0], pose[0] + car.front * c], [pose[1], pose[1] + car.front * s])
        for i in range(5):
            ts, rs = a["tof"][i]
            j = int(np.searchsorted(ts, t, side="right")) - 1
            if j >= 0 and t - ts[j] < 0.1 and np.isfinite(rs[j]):
                sx, sy, sa = self.tof.sensor_pose(pose, i)
                hx, hy = sx + rs[j] * np.cos(sa), sy + rs[j] * np.sin(sa)
                a["rays"][i].set_data([sx, hx], [sy, hy])
                a["hits"][i].set_data([hx], [hy])
            else:
                a["rays"][i].set_data([], [])
                a["hits"][i].set_data([], [])
        n_ev = int(np.searchsorted(a["ev_t"], t, side="right"))
        last_ev = self.res.events[n_ev - 1][1] if n_ev else "-"
        a["text"].set_text(f"t = {L['t'][k]:5.2f} s   v = {L['v'][k]:+.2f} m/s   ω = {L['w'][k]:+.2f} rad/s\n"
                           f"trạng thái: {MODE_NAMES[int(L['mode'][k])]}\n"
                           f"sự kiện: {textwrap.shorten(last_ev, 60)}")
        for cur in self.cursors:
            cur.set_xdata([t, t])
        if self.interactive:
            self.fig.canvas.draw_idle()

    def toggle_play(self):
        if self.anim is None:
            self.set_status("Chưa có lượt chạy để phát lại (Monte-Carlo không phát lại được)", err=True)
            return
        if self.playing:
            self.stop_play()
            return
        if self.s_t.val >= self.s_t.valmax - 1e-6:
            self.s_t.set_val(0.0)
        self.playing = True
        self.b_play.label.set_text("⏸ Dừng")
        self.timer.start()

    def stop_play(self):
        if self.playing:
            self.timer.stop()
            self.playing = False
            self.b_play.label.set_text("▶ Phát")

    def _tick(self):
        t = self.s_t.val + FRAME_S * self.play_speed
        if t >= self.s_t.valmax:
            t = self.s_t.valmax
            self.stop_play()
        self.s_t.set_val(t)

    def save_animation(self, path=None, fps=15, speed=None):
        """Xuất hoạt hình lượt chạy hiện tại ra GIF."""
        if self.anim is None:
            if self.interactive:
                self.set_status("Chưa có lượt chạy để xuất GIF", err=True)
            return None
        self.stop_play()
        speed = self.play_speed if speed is None else speed
        if path is None:
            os.makedirs(SIM_OUT, exist_ok=True)
            bx = "none" if self.box is None else f"{self.box[0]:.0f}_{self.box[1]:.0f}"
            path = os.path.join(SIM_OUT, f"{time.strftime('%Y%m%d_%H%M%S')}_box{bx}_seed{self.seed}.gif")
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        if self.interactive:
            self.set_status("Đang xuất GIF ...")
        T = float(self.res.log["t"][-1])
        times = np.append(np.arange(0.0, T, speed / fps), T)
        # GIF chỉ gồm bản đồ, vẽ trên hình riêng (vẽ lại cả cửa sổ mỗi khung hình rất chậm)
        fig = plt.figure(figsize=(7.5, 6.4))
        ax = fig.add_axes([0.06, 0.04, 0.92, 0.94])
        draw_base(ax, self.ctx.m)
        ax.plot(self.nom[:, 1], self.nom[:, 2], color="#777", lw=1.2, label="Quỹ đạo chính")
        if self.box is not None:
            ax.add_patch(Rectangle((self.box[0] - 50, self.box[1] - 50), 100, 100, fc="#ffc845", ec="#a05a00",
                                   lw=1.5, zorder=4))
        saved = (self.ax, self.dyn, self.anim, self.cursors, self.interactive)
        self.ax, self.dyn, self.cursors, self.interactive = ax, [], [], False
        try:
            self._make_anim_artists()
            ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
            anim = FuncAnimation(fig, lambda i: self.draw_frame(times[i]), frames=len(times), blit=False)
            anim.save(path, writer=PillowWriter(fps=fps), dpi=80)
        finally:
            self.ax, self.dyn, self.anim, self.cursors, self.interactive = saved
            plt.close(fig)
        self.draw_frame(T)
        if self.interactive:
            self.s_t.set_val(T)
            self.set_status(f"Đã xuất GIF: {os.path.relpath(path)}")
        return path

    def _draw_info(self):
        self.ax_info.clear()
        self.ax_info.axis("off")
        lines = []
        if self.mc:
            S = [summary(r) for r in self.mc]
            n = len(S)
            fin = sum(s["finished"] for s in S)
            clean = sum(s["finished"] and not s["collisions"] and not s["wheel_out"] for s in S)
            sc = np.array([s["score"] for s in S])
            el = np.array([s["max_e_lat_mm"] for s in S])
            lines += [f"MONTE-CARLO {n} lần  |  box: {self._box_str()}  |  nhiễu ×{self.noise:.1f}  v_max {self.v_max:.2f}",
                      f"Về đích: {fin}/{n}   chạy sạch: {clean}/{n}   va chạm: {sum(s['collisions'] > 0 for s in S)}/{n}"
                      f"   bánh ra ngoài: {sum(s['wheel_out'] > 0 for s in S)}/{n}",
                      f"Điểm: TB {sc.mean():.1f}  min {sc.min():.1f}  max {sc.max():.1f}",
                      f"Lệch ngang max: TB {el.mean():.1f} mm  P95 {np.percentile(el, 95):.1f} mm",
                      "Né: " + ", ".join(f"{k}×{v}" for k, v in _count([s['detour'] for s in S]).items())]
        elif self.res is not None:
            s = summary(self.res)
            r = self.res
            dt = self.detour
            plan_txt = "-" if dt is None else ("không cần né (cách {:.0f} mm)".format(dt["clear_R"]) if not dt["need"]
                                               else ("có nhánh né, tách tại s = {:.2f} m".format(
                                                   self.nom[dt["i_split"], 0] / 1000) if dt["ok"]
                                                     else "KHÔNG lập được nhánh né"))
            lines += [f"ĐIỂM: {r.score['total']:.1f}   |   " + "  •  ".join(r.score["detail"]),
                      f"Box: {self._box_str()}  |  Lập trước: {plan_txt}  |  Trên xe: {s['detour']}",
                      f"Về đích: {'%.2f s' % s['t_finish'] if s['finished'] else 'KHÔNG'}   "
                      f"lệch ngang max {s['max_e_lat_mm']:.1f} mm   sai số ước lượng max {s['max_e_est_mm']:.1f} mm",
                      f"Cách box min {s['min_clr_box_mm']:.1f} mm   cách biên min {s['min_clr_wall_mm']:.1f} mm   "
                      f"va chạm {s['collisions']}   bánh ra ngoài {s['wheel_out']}   " + (
                          f"sai số ước lượng tâm box {s['box_est_err_mm']:.0f} mm"
                          if s["box_est_err_mm"] is not None else
                          ("box chưa được phát hiện" if self.box is not None else "")),
                      "Sự kiện: " + ("; ".join(f"{t:.2f}s {m}" for t, m in r.events[-5:]) or "-")]
        wrapped = []
        for ln in lines:
            wrapped += textwrap.wrap(ln, 105, subsequent_indent="  ") or [""]
        self.ax_info.text(0, 1, "\n".join(wrapped[:9]), va="top", ha="left", fontsize=8, family="monospace")

    def _box_str(self):
        return "không có" if self.box is None else f"({self.box[0]:.0f}, {self.box[1]:.0f})"

    # ---------- lưu ----------
    def save(self, tag=None):
        os.makedirs(SIM_OUT, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        bx = "none" if self.box is None else f"{self.box[0]:.0f}_{self.box[1]:.0f}"
        base = os.path.join(SIM_OUT, tag or f"{ts}_box{bx}_seed{self.seed}")
        self.fig.savefig(base + ".png", dpi=110)
        if self.res is not None and not self.mc:
            L = self.res.log
            keys = list(L.keys())
            with open(base + "_log.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(keys)
                w.writerows(zip(*[np.round(L[k], 4) for k in keys]))
            names = [t["name"] for t in self.ctx.cfg["robot"]["tof"]]
            with open(base + "_tof.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t", "sensor", "range_mm"])
                for t, i, r in self.res.tof:
                    w.writerow([f"{t:.3f}", names[int(i)], "" if np.isnan(r) else f"{r:.1f}"])
        with open(base + "_summary.txt", "w", encoding="utf-8") as f:
            runs = self.mc or ([self.res] if self.res else [])
            for i, r in enumerate(runs):
                f.write(f"run {i}: {summary(r)}\n  {r.score}\n")
                for t, msg in r.events:
                    f.write(f"  {t:.3f}s {msg}\n")
        if self.interactive:
            self.set_status(f"Đã lưu: {os.path.relpath(base)}.*")
        return base


def _count(xs):
    out = {}
    for x in xs:
        out[x] = out.get(x, 0) + 1
    return out


def sweep(ctx, nom, seed, noise, v_max, use_ekf, step=50.0, n_random=0, use_recovery=True):
    """Quét vị trí box (liên tục) trong ô 4/5: lưới bước `step` mm, hoặc `n_random` vị trí ngẫu nhiên."""
    m = ctx.m
    tiles = [t for t in m.tiles if t["id"] in m.random_tiles]
    x0, x1 = min(t["x0"] for t in tiles) + 50, max(t["x0"] for t in tiles) + m.tile - 50
    y0, y1 = min(t["y0"] for t in tiles) + 50, max(t["y0"] for t in tiles) + m.tile - 50
    if n_random:
        rng = np.random.default_rng(seed + 1000)
        cells = []
        while len(cells) < n_random:
            c = (float(rng.uniform(x0, x1)), float(rng.uniform(y0, y1)))
            if m.box_fits(*c):
                cells.append(c)
    else:
        cells = [(float(x), float(y)) for x in np.arange(x0, x1 + 1, step) for y in np.arange(y0, y1 + 1, step)
                 if m.box_fits(x, y)]
    app = App(ctx, nom, interactive=False)
    app.seed, app.noise, app.use_ekf = seed, noise, use_ekf
    app.use_recovery = app.use_recovery and use_recovery
    if v_max:
        app.v_max = v_max
    rows = []
    for i, c in enumerate(cells):
        app.box = c
        app._plan()
        r = app._sim(seed)
        s = summary(r)
        dt = app.detour
        rows.append({"x": c[0], "y": c[1], "need_detour": dt["need"], "detour_ok": dt["ok"],
                     "on_robot": s["detour"], "finished": s["finished"], "t": s["t_finish"],
                     "score": round(s["score"], 1), "collisions": s["collisions"], "wheel_out": s["wheel_out"],
                     "max_e_lat_mm": round(s["max_e_lat_mm"], 1), "min_clr_box_mm": round(s["min_clr_box_mm"], 1),
                     "box_est_err_mm": None if s["box_est_err_mm"] is None else round(s["box_est_err_mm"], 1)})
        err = "-" if s["box_est_err_mm"] is None else f"{s['box_est_err_mm']:.0f}mm"
        print(f"[{i + 1}/{len(cells)}] box ({c[0]:.0f},{c[1]:.0f}): cần né={dt['need']} lập được={dt['ok']} "
              f"trên xe={s['detour']} về đích={s['finished']} điểm={s['score']:.1f} sai số tâm={err}")
    os.makedirs(SIM_OUT, exist_ok=True)
    with open(os.path.join(SIM_OUT, "sweep.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    fig, ax = plt.subplots(figsize=(11, 9))
    draw_base(ax, m)
    ax.plot(nom[:, 1], nom[:, 2], color="#555", lw=1.2)
    for rw in rows:
        bad = not rw["finished"]
        pen = rw["collisions"] or rw["wheel_out"]
        col = "#e74c3c" if bad else ("#f39c12" if pen else ("#8e44ad" if rw["on_robot"].startswith("cứu") else
                                                           ("#27ae60" if rw["need_detour"] else "#a3d9a5")))
        # mỗi điểm là TÂM box ngẫu nhiên tại vị trí đó
        ax.scatter(rw["x"], rw["y"], s=60 if n_random else (step / 100 * 9) ** 2, marker="s", color=col,
                   alpha=0.85, edgecolors="none", zorder=4)
    ax.set_title("Tâm box ngẫu nhiên: xanh nhạt = không cần né, xanh = né thành công, tím = về đích nhờ lùi/xoay,\n"
                 "cam = về đích nhưng bị trừ, đỏ = không về đích", fontsize=9)
    fig.savefig(os.path.join(SIM_OUT, "sweep.png"), dpi=120)
    n = len(rows)
    errs = [r["box_est_err_mm"] for r in rows if r["box_est_err_mm"] is not None]
    if errs:
        print(f"Sai số ước lượng tâm box: TB {np.mean(errs):.1f} mm, P95 {np.percentile(errs, 95):.1f} mm, "
              f"max {np.max(errs):.1f} mm")
    print(f"\nTổng: {n} vị trí | cần né {sum(r['need_detour'] for r in rows)} | "
          f"về đích {sum(r['finished'] for r in rows)} | bị trừ {sum(bool(r['collisions'] or r['wheel_out']) for r in rows)}")
    print(f"Đã lưu {os.path.relpath(SIM_OUT)}/sweep.csv và sweep.png")


def main():
    a = ARGS
    ctx = Context()
    nom, _ = nominal_cached(ctx)
    if a.sweep or a.random:
        sweep(ctx, nom, a.seed, a.noise, a.vmax, not a.no_ekf, a.step, a.random, not a.no_recovery)
        return 0
    interactive = not (a.headless or a.mc or a.animate)
    app = App(ctx, nom, interactive=interactive)
    app.seed, app.noise = a.seed, a.noise
    app.use_ekf = not a.no_ekf
    app.use_recovery = app.use_recovery and not a.no_recovery
    if a.vmax:
        app.v_max = a.vmax
    if a.box:
        x, y = a.box
        if not ctx.m.box_fits(x, y):
            print(f"Box ({x:.0f}, {y:.0f}) không hợp lệ (phải nằm gọn trong ô 4/5, không chồng box tĩnh)")
            return 2
        app.box = (x, y)
    if interactive:
        if a.box is None:
            app.run()
        else:
            app.set_box(app.box)
        plt.show()
        return 0
    if a.mc:
        runs = app.run_mc(a.mc)
        for i, r in enumerate(runs):
            print(f"seed {a.seed + i}: {summary(r)}")
    else:
        app.run()
        print(summary(app.res))
        print(app.res.score)
        for t, msg in app.res.events:
            print(f"  {t:.3f}s {msg}")
    if a.animate:
        print("Đang xuất GIF ...")
        print("Đã xuất:", os.path.relpath(app.save_animation(a.animate, speed=a.anim_speed)))
        return 0
    print("Đã lưu:", os.path.relpath(app.save()) + ".*")
    return 0


if __name__ == "__main__":
    sys.exit(main())
