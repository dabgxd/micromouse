"""Lập quỹ đạo cho xe dò mê cung.

Chạy:  tools/.venv/bin/python tools/planner/plan.py
Kết quả nằm trong tools/planner/out/.
"""
import os
import sys

import numpy as np

from export import plot, write_header, write_recovery
from partitions import box_partition, build_lines, crossing_index, recovery_table
from pipeline import OUT, Context, plan_nominal, speed_for


def main():
    os.makedirs(OUT, exist_ok=True)
    ctx = Context()
    m, car, cl = ctx.m, ctx.car, ctx.cl

    traj, raw, ok = plan_nominal(ctx)
    db, dw = car.clearance(m, traj[:, 1:4])

    v, t = speed_for(ctx, traj)
    print(f"[3] Vận tốc: dài {traj[-1, 0] / 1000:.2f} m, ước tính {t[-1]:.1f} s "
          f"(điểm tốc độ ≈ {(240 - t[-1]) / 6:.1f})")

    lines = build_lines(m)
    cross = crossing_index(car, traj, lines)
    missing = [ln.k for ln, c in zip(lines, cross) if c is None]
    box_line = {bid: box_partition(m, lines, x, y) for bid, x, y in m.boxes}
    rec = recovery_table(m, car, traj, lines, cross, cl)
    bad = [(r["k"], r["clear_box"], r["clear_wall"]) for r in rec
           if r and (r["clear_box"] < cl["box_soft_mm"] or r["clear_wall"] < cl["wall_soft_mm"])]
    print(f"[4] Vạch chia phần: {len(lines) - len(missing)}/{len(lines)} vạch được vượt"
          + (f", THIẾU: {missing}" if missing else ""))
    for k, rb, rw in bad:
        print(f"    !! Pose chỉnh xe tại vạch {k} sát vật cản (box {rb:.0f} mm, biên {rw:.0f} mm)")

    write_header(os.path.join(OUT, "trajectory_data.h"), m, car, traj, v, lines, cross, rec, t[-1])
    write_recovery(os.path.join(OUT, "recovery_table.md"), m, lines, rec, box_line)
    np.savetxt(os.path.join(OUT, "trajectory.csv"), np.column_stack([traj, v, t]), delimiter=",",
               header="s_mm,x_mm,y_mm,theta_rad,kappa_1pmm,v_mps,t_s", fmt="%.5f")
    plot(os.path.join(OUT, "trajectory.png"), m, car, traj, v, t, db, dw, lines, rec, raw=raw)
    print(f"[5] Đã xuất: {os.path.relpath(OUT)}/trajectory_data.h, recovery_table.md, "
          f"trajectory.csv, trajectory.png")
    return 0 if ok and not missing else 1


if __name__ == "__main__":
    sys.exit(main())
