# Planner: lập quỹ đạo cho xe

## Cài đặt (một lần)

```sh
python3 -m venv tools/.venv
tools/.venv/bin/pip install numpy scipy matplotlib
```

## Chạy

```sh
tools/.venv/bin/python tools/planner/plan.py
```

Mất khoảng 10–60 giây. Kết quả nằm trong `tools/planner/out/`:

| File | Nội dung |
|---|---|
| `trajectory.png` | Map, quỹ đạo (màu theo vận tốc), đường bao thân xe, vạch chia phần, pose chỉnh xe (đỏ chấm), đồ thị v(s), độ cong và khoảng cách an toàn |
| `trajectory_data.h` | Dữ liệu cho firmware: quỹ đạo mỗi 10 mm, 24 vạch chia phần, pose chỉnh xe, toạ độ 17 box |
| `recovery_table.md` | Bảng chỉnh xe để in ra giấy |
| `trajectory.csv` | Quỹ đạo dạng bảng (để mô phỏng hoặc vẽ) |

Lệnh trả về mã 0 nếu quỹ đạo hợp lệ, mã 1 nếu vi phạm biên an toàn hoặc có vạch không được vượt.

## Sửa dữ liệu

- **`map.json`**: toạ độ tâm 17 box (mm). Đo sân thật rồi sửa ở đây.
- **`config.json`**:
  - `robot`: kích thước xe tính từ tâm trục bánh.
  - `clearance`: khoảng cách an toàn. `hard` là giới hạn cứng, không được vi phạm. `soft` là mục tiêu, nhỏ hơn thì bị phạt chi phí.
  - `start`: dải vị trí xuất phát mà tool được phép chọn.
  - `astar.r_min_mm`: bán kính cua nhỏ nhất.
  - `speed`: vận tốc tối đa và các giới hạn gia tốc.

## Thuật toán

1. **Trường khoảng cách** (`geometry.py`): tính khoảng cách tới box và tới biên trên lưới 5 mm. Thân xe được mô hình bằng các điểm trên chu vi hình chữ nhật 137.5 × 87 mm.
2. **Hybrid A\*** (`hybrid_astar.py`): tìm đường trong không gian (x, y, θ), chỉ đi tiến, với các cung có bán kính ≥ `r_min`. Heuristic là Dijkstra 2D tính từ vùng đích. Thử đồng thời mọi vị trí xuất phát dọc vạch.
3. **Làm mượt** (`smooth.py`): tối ưu L-BFGS cân bằng giữa độ mượt, khoảng cách an toàn, giới hạn độ cong và độ lệch so với đường gốc. Sau đó nội suy cubic spline và lấy mẫu mỗi 10 mm.
4. **Profile vận tốc**: giới hạn bởi gia tốc ngang, rồi duyệt xuôi (tăng tốc) và duyệt ngược (giảm tốc).
5. **Vạch chia phần** (`partitions.py`): tìm thời điểm cả 4 góc xe đã qua từng vạch. Pose chỉnh xe được chọn sao cho đuôi xe sát vạch, thân song song làn, vị trí ngang gần quỹ đạo nhất mà vẫn an toàn.

---

# Mô phỏng tương tác (`sim_gui.py`)

```sh
tools/.venv/bin/python tools/planner/sim_gui.py
```

- **Click trái** vào ô 4/5 để đặt box ngẫu nhiên ở **vị trí bất kỳ** (không theo lưới; có thể vắt ngang ranh giới ô 4/5). Tool lập nhánh né, mô phỏng một lượt chạy và chấm điểm.
- **Click phải** để xoá box.
- **▶ Mô phỏng**: chạy lại với seed, nhiễu và v_max đang chọn.
- **Monte-Carlo ×20**: chạy 20 lần với seed khác nhau, cho ra tỉ lệ về đích, tỉ lệ va chạm và phân bố điểm.
- **Lưu kết quả**: ghi PNG, log CSV (pose thật/ước lượng, sai số, khoảng cách an toàn), CSV số đo ToF và tóm tắt vào `out/sim/`.
- **▶ Phát / ⏸ Dừng**: phát lại lượt chạy dạng hoạt hình. Hiển thị xe thật (khối xanh), vị trí EKF ước lượng (viền đỏ nét đứt), 5 tia ToF với điểm đo, vệt đường đã đi, cùng bảng thời gian / vận tốc / trạng thái (tiến, lùi, xoay tại chỗ, dừng) / sự kiện mới nhất. Các biểu đồ bên phải có con trỏ thời gian chạy theo.
- **Thanh `t (s)`**: kéo để tua tới thời điểm bất kỳ. **Tốc độ phát ×**: 0.25–4 lần thời gian thực.
- **GIF**: xuất hoạt hình (chỉ phần bản đồ) vào `out/sim/`.
- **Tuỳ chọn**: bật/tắt EKF, so sánh với quỹ đạo toàn cục (khi biết trước box), cho phép lùi / xoay khi kẹt. Thanh trượt: v_max, hệ số nhiễu, seed.

Chạy không giao diện:

```sh
tools/.venv/bin/python tools/planner/sim_gui.py --box 1250 350 --headless --seed 2
tools/.venv/bin/python tools/planner/sim_gui.py --box 1250 350 --mc 30 --noise 1.5
tools/.venv/bin/python tools/planner/sim_gui.py --sweep --step 50   # quét tâm box trên lưới 50 mm -> out/sim/sweep.png/csv
tools/.venv/bin/python tools/planner/sim_gui.py --random 100         # quét 100 vị trí box ngẫu nhiên liên tục
tools/.venv/bin/python tools/planner/sim_gui.py --box 1304 674 --animate run.gif --anim-speed 2   # xuất hoạt hình GIF
```

## Mô phỏng những gì

| Thành phần | Mô hình |
|---|---|
| Xe | Động học vi sai. Motor có trễ bậc 1 (`motor_tau_s`). Bán kính bánh thật lệch so với danh định, có trượt bánh |
| Encoder | Lượng tử hoá theo `encoder_cpr_wheel` |
| Gyro | Bias dư, sai số hệ số, nhiễu trắng |
| ToF | 5 tia trong nón 25°, trả về mục tiêu gần nhất. Nhiễu tỉ lệ theo khoảng cách. Có giá trị đột biến |
| Định vị (`estimator.py`) | Odometry + gyro → EKF. Cập nhật bằng ToF khi cả nón rơi trên cùng một mặt box đã biết, lọc theo khoảng cách Mahalanobis |
| Phát hiện box lạ | Số đo (≤ 600 mm) ngắn hơn map ≥ 50 mm là bất thường. Gom các số đo gần nhau, rồi **khớp hình vuông 100 mm ở vị trí liên tục**: điểm gần nhất trên cung nón đo nằm trên mặt box, phần tia phía trước phải trống, sai số có chặn trên. Chỉ xác nhận khi độ bất định ≤ `max_spread_mm` và đã nhìn từ ≥ 2 vị trí. Ước lượng được tinh chỉnh liên tục, và xe lập lại nhánh khi tâm box dịch > `replan_shift_mm` |
| Né | Phát hiện đủ sớm thì chuyển sang nhánh lập sẵn. Phát hiện muộn thì lập nhánh từ vị trí hiện tại. M0 < 70 mm hai lần liên tiếp thì phanh |
| Điều khiển | Kanayama (feedforward theo độ cong + phản hồi e_y, e_θ), 200 Hz |
| Chấm điểm | Theo Điều 3: vạch chia phần, xô box, bánh ra ngoài, tốc độ, chạy sạch |

Mọi tham số nằm trong `config.json` (`sim`, `control`, `ekf`, `detect`).

## Quỹ đạo cứu: lùi / xoay tại chỗ (`config.json` → `recovery`)

Ở một số vị trí (đặc biệt là lối vào ô 5, sát góc trong chữ U), box ngẫu nhiên chỉ lộ ra sau khi xe đã cua, nên không còn đường né nào nếu chỉ đi tiến. Khi đó:

1. **Kích hoạt**: không lập được nhánh né đi tiến, hoặc xe phanh khẩn cấp quá `trigger_stop_s`.
2. **Xe dừng hẳn**, rồi lập quỹ đạo cứu bằng Hybrid A\* có thêm động tác **lùi theo cung** (phạt `w_rev`, đổi chiều phạt `w_switch_mm`) và **xoay tại chỗ** từng bước `spin_step_deg`. Lưới tìm kiếm thô hơn (`recovery.astar`, heuristic có trọng số `h_weight`): chỉ khoảng 1.000–5.000 node, đủ nhẹ để chạy trên MCU. Biên an toàn cứng được nới xuống bằng khoảng cách hiện tại nếu xe đang đứng sát box.
3. Kết quả là chuỗi đoạn `xoay` / `lùi` / `tiến`. Đoạn tiến cuối cùng nhập lại quỹ đạo chính sau box, hoặc đi thẳng qua vạch đích.
4. Trên xe: khi lùi, bám quỹ đạo bằng "xe ảo quay 180°" rồi đảo dấu vận tốc. Khi xoay, dùng điều khiển P theo góc yaw. Phanh khẩn cấp chỉ áp dụng khi đi tiến, với ngưỡng nhỏ hơn (`stop_front_mm`) trong lúc cứu.
5. Nếu tâm box được cập nhật lệch hơn `replan_shift_mm` trong lúc đang cứu, xe dừng lại và lập lại quỹ đạo cứu. Tối đa `max_recoveries` lần mỗi lượt.

## Độ bất định vị trí box

- Ước lượng đầu tiên (ít số đo, thường chỉ nhìn từ một phía) có thể sai vài cm. Khi lập nhánh né, box được **phóng to** thêm `min(inflate_max_mm, spread + inflate_k_mm / sqrt(số đo))` mỗi phía.
- Ước lượng chỉ được thay bằng một cụm số đo khác khi cụm đó có ≥ 2×`hit_thresh` số đo. Khi đã có ≥ `lock_reads` số đo thì ước lượng bị **khoá** (luật chỉ có một box ngẫu nhiên).
