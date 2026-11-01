"""Xuất trajectory.h cho firmware, bảng chỉnh xe và hình vẽ."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

from partitions import car_corners, describe_line  # noqa: E402


def write_header(path, m, car, traj, v, lines, cross, rec, t_total):
    L = []
    w = L.append
    w("/* Tự động sinh bởi tools/planner/plan.py - KHÔNG SỬA TAY. */")
    w("#ifndef TRAJECTORY_DATA_H")
    w("#define TRAJECTORY_DATA_H")
    w("")
    w("#include <stdint.h>")
    w("")
    w("/* Đơn vị: mm, rad, m/s. Gốc: góc dưới-trái ô 1; pose là tâm trục bánh. */")
    w("typedef struct { float s, x, y, th, k, v; } traj_pt_t;")
    w("typedef struct { float ax, ay, nx, ny; int16_t traj_idx; float rx, ry, rth; } part_line_t;")
    w("typedef struct { float x, y; } box_t;")
    w("")
    w(f"#define TRAJ_N            {len(traj)}")
    w(f"#define TRAJ_LENGTH_MM    {traj[-1, 0]:.1f}f")
    w(f"#define TRAJ_EST_TIME_S   {t_total:.2f}f")
    w(f"#define PART_N            {len(lines)}")
    w(f"#define BOX_N             {len(m.boxes)}")
    w(f"#define BOX_HALF_MM       {m.box / 2:.1f}f")
    w("")
    w("static const traj_pt_t TRAJ[TRAJ_N] = {")
    for (s, x, y, th, k), vv in zip(traj, v):
        w(f"  {{{s:.1f}f, {x:.1f}f, {y:.1f}f, {th:.5f}f, {k:.6f}f, {vv:.3f}f}},")
    w("};")
    w("")
    w("/* Vạch chia phần: điểm a, pháp tuyến hướng đi, chỉ số mẫu khi cả xe qua vạch, pose đặt lại khi chỉnh xe. */")
    w("static const part_line_t PART_LINES[PART_N] = {")
    for ln, idx, r in zip(lines, cross, rec):
        rx, ry, rth = r["pose"] if r else (0.0, 0.0, 0.0)
        w(f"  {{{ln.a[0]:.1f}f, {ln.a[1]:.1f}f, {ln.n[0]:.4f}f, {ln.n[1]:.4f}f, "
          f"{-1 if idx is None else idx}, {rx:.1f}f, {ry:.1f}f, {rth:.5f}f}}, /* vạch {ln.k}, ô {ln.tile} */")
    w("};")
    w("")
    w("static const box_t STATIC_BOXES[BOX_N] = {")
    for bid, x, y in m.boxes:
        w(f"  {{{x:.1f}f, {y:.1f}f}}, /* #{bid} */")
    w("};")
    w("")
    w("#endif /* TRAJECTORY_DATA_H */")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


def write_recovery(path, m, lines, rec, box_line):
    L = ["# Bảng chỉnh xe", "",
         "Đặt **đuôi xe sát sau vạch k** (vạch cuối cùng mà cả xe đã qua), thân xe **song song làn**,",
         "tâm xe cách mốc một khoảng như bảng, rồi bấm KEY.", "",
         "| Vạch k | Ô | Tâm xe cách mốc | Mốc đo | Box được tính khi qua vạch này |",
         "|---|---|---|---|---|"]
    for ln, r in zip(lines, rec):
        boxes = ", ".join(f"#{b}" for b, k in box_line.items() if k == ln.k) or "-"
        if r is None:
            L.append(f"| {ln.k} | {ln.tile} | (quỹ đạo không qua) | | {boxes} |")
        else:
            L.append(f"| {ln.k} | {ln.tile} | **{r['along_mm'] / 10:.1f} cm** | {describe_line(m, ln)} | {boxes} |")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")


def plot(path, m, car, traj, v, t, clr_box, clr_wall, lines, rec, raw=None, random_box=None):
    fig = plt.figure(figsize=(13, 14))
    ax = fig.add_axes([0.05, 0.3, 0.85, 0.66])
    ax.set_aspect("equal")
    ax.add_patch(Polygon(m.polygon, closed=True, fc="#f3f3ee", ec="black", lw=2))
    ax.add_patch(Rectangle((600, 0), 600, 600, fc="#f6d5d0", ec="none", hatch="//"))
    for tl in m.tiles:
        ax.add_patch(Rectangle((tl["x0"], tl["y0"]), m.tile, m.tile, fc="none", ec="#bbb", lw=0.8))
        ax.text(tl["x0"] + 15, tl["y0"] + m.tile - 35, f"Ô {tl['id']}", color="#888", fontsize=10)
    for ln in lines:
        ax.plot(*np.array([ln.a, ln.b]).T, color="#9b6dd6", lw=0.8, ls="--")
        mid = (ln.a + ln.b) / 2
        ax.text(*mid, str(ln.k), color="#9b6dd6", fontsize=7, ha="center", va="center")
    for bid, x, y in m.boxes:
        ax.add_patch(Rectangle((x - 50, y - 50), 100, 100, fc="#ccc", ec="#444", lw=1.2))
        ax.text(x, y, str(bid), ha="center", va="center", fontsize=8)
    if random_box is not None:
        x, y = random_box
        ax.add_patch(Rectangle((x - 50, y - 50), 100, 100, fc="#ffd27a", ec="#a60", lw=1.2))
    ax.plot([0, 600], [0, 0], color="green", lw=3)
    ax.plot([1200, 1800], [0, 0], color="black", lw=3, ls=(0, (2, 2)))

    if raw is not None:
        ax.plot(raw[:, 0], raw[:, 1], color="#999", lw=1, ls=":", label="Hybrid A* (thô)")
    for i in range(0, len(traj), 15):
        cs = car_corners(car, traj[i, 1:4])
        ax.add_patch(Polygon(cs, closed=True, fc="none", ec="#4a7bd0", lw=0.4, alpha=0.6))
    sc = ax.scatter(traj[:, 1], traj[:, 2], c=v, cmap="viridis", s=4, zorder=5)
    fig.colorbar(sc, cax=fig.add_axes([0.92, 0.4, 0.015, 0.45]), label="v (m/s)")
    for r in rec:
        if r:
            cs = car_corners(car, r["pose"])
            ax.add_patch(Polygon(cs, closed=True, fc="none", ec="#d04a4a", lw=0.6, ls=":"))
    ax.set_xlim(-60, 1860)
    ax.set_ylim(-320, 1260)
    ax.set_title(f"Quỹ đạo: dài {traj[-1, 0] / 1000:.2f} m, ước tính {t[-1]:.1f} s, "
                 f"cách box tối thiểu {clr_box.min():.0f} mm, cách biên tối thiểu {clr_wall.min():.0f} mm")
    if raw is not None:
        ax.legend(loc="upper right")

    s = traj[:, 0] / 1000
    a1 = fig.add_axes([0.05, 0.04, 0.42, 0.18])
    a1.plot(s, v, label="v (m/s)")
    a1.plot(s, np.abs(traj[:, 4]) * 1000 / 10, label="|κ|/10 (1/m)", alpha=0.6)
    a1.set_xlabel("s (m)"); a1.legend(); a1.grid(alpha=0.3)
    a2 = fig.add_axes([0.54, 0.04, 0.42, 0.18])
    a2.plot(s, clr_box, label="cách box (mm)")
    a2.plot(s, clr_wall, label="cách biên (mm)")
    a2.axhline(50, color="gray", ls=":"); a2.axhline(25, color="red", ls=":")
    a2.set_ylim(-10, 200); a2.set_xlabel("s (m)"); a2.legend(); a2.grid(alpha=0.3)
    fig.savefig(path, dpi=130)
    plt.close(fig)
