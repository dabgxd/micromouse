# Micromouse – xe tự hành né vật cản

Đồ án môn ELT3240 Nhập môn Hệ thống nhúng (UET-VNU).
Xe 2 bánh N20 + TB6612, 5 cảm biến khoảng cách VL53L0X, MPU6050, giao tiếp UART (USB-TTL hoặc HC-05).
Vi điều khiển STM32F411CEU6 (Black Pill), build bằng PlatformIO + STM32Cube HAL.

## Cấu trúc thư mục

```
micromouse/
├── firmware/                     project PlatformIO (mở thư mục này bằng VS Code + PlatformIO)
│   ├── platformio.ini            2 môi trường build: main (thi đấu) và hwtest (test phần cứng)
│   ├── firmware_micromouse.ioc   cấu hình CubeMX: chân, clock, ngoại vi
│   ├── src/                      code CubeMX sinh ra (main.c, ngắt, MSP...)
│   │   └── hwtest/               ứng dụng test phần cứng, chỉ build trong env hwtest
│   ├── Inc/                      header CubeMX (main.h chứa tên chân)
│   ├── lib/                      driver và bộ điều khiển dùng chung cho test và code chính
│   │   ├── common/               robot_config.h (thông số cơ khí), timebase (đếm chu kỳ CPU)
│   │   ├── tb6612/               driver động cơ
│   │   ├── encoder/              đếm xung 2 bánh (TIM3, TIM4)
│   │   ├── tof/                  driver VL53L0X + quản lý dãy 5 cảm biến
│   │   ├── mpu/                  driver MPU6050 + ước lượng góc (yaw, tự bù trôi, gỡ treo bus)
│   │   ├── pid/                  bù trước động cơ, PID vị trí bánh xe, profile hình thang
│   │   └── motion/               chuyển động kín vòng: đi N xung, giữ vận tốc, chạy thẳng, quay tại chỗ
│   └── Drivers/                  HAL + CMSIS (CubeMX chép vào, không sửa tay)
├── tools/
│   ├── hwtest_web/               bảng điều khiển test chạy trên trình duyệt
│   └── planner/                  lập quỹ đạo và mô phỏng (Python, xem README riêng)
├── docs/                         luật thi, bản vẽ xe, bản đồ sân, báo cáo giữa kỳ, kịch bản thuyết trình
├── cad_files/                    file thiết kế cơ khí 3D
└── schematic/                    sơ đồ mạch (KiCad)
```

Code chính sau này đặt trong `firmware/src/` (ví dụ `firmware/src/app/`) và gọi các thư viện trong
`firmware/lib/` giống hệt cách `firmware/src/hwtest/` đang dùng.

## Sơ đồ chân

| Chức năng | Chân | Ngoại vi |
|---|---|---|
| Động cơ phải (TB6612 kênh A) | PWMA PA8, AIN1 PB12, AIN2 PB13 | TIM1_CH1 |
| Động cơ trái (TB6612 kênh B) | PWMB PA9, BIN1 PB14, BIN2 PB15 | TIM1_CH2 |
| TB6612 STBY | PA10 | GPIO |
| Encoder trái A / B | PA7 / PA6 | TIM3 |
| Encoder phải A / B | PB7 / PB6 | TIM4 |
| ToF (I2C1) SCL / SDA | PB8 / PB9 | I2C1 400 kHz |
| ToF XSHUT L90, L45, M0, R45, R90 | PA1, PA4, PA5, PB0, PB1 | GPIO |
| MPU6050 (I2C2) SCL / SDA, INT | PB10 / PB3, PB5 | I2C2 400 kHz |
| UART (TTL / HC-05) TX / RX | PA2 / PA3 | USART2 38400 8N1, DMA |
| LED | PC13 | nháy 1 Hz khi firmware test chạy |

Động cơ và encoder bên phải lắp đối xứng: thư viện đã đảo chiều bên trong, nên số dương luôn là chạy tiến.

## Build và nạp

```sh
cd firmware
pio run -e hwtest -t upload   # firmware test phần cứng
pio run -e main -t upload     # firmware chính (hiện mới có khởi tạo CubeMX)
```

Nạp bằng ST-Link (SWDIO PA13, SWCLK PA14). Nút Upload trong VS Code nạp env đang chọn ở thanh trạng thái;
mặc định là `main`, nên muốn test thì chọn `env:hwtest`.

## Test phần cứng bằng web

1. Nạp `hwtest`. Đèn PC13 nháy đều là firmware test đang chạy.
2. Nối USB-TTL (RX → PA2, TX → PA3, chung GND) hoặc ghép đôi HC-05.
3. Mở web:
   ```sh
   tools/hwtest_web/serve.sh
   ```
   Script tự bật server (nếu chưa chạy) và mở http://localhost:8000 trong Chrome. Chạy lại bao nhiêu lần
   cũng được: server đã chạy sẵn thì script chỉ mở trình duyệt. Trên Windows/Linux dùng
   `cd tools/hwtest_web && python3 -m http.server 8000`. Cần **Chrome** hoặc **Edge** (Web Serial).
4. Chọn baud 38400, bấm **Kết nối**, chọn cổng. Làm lần lượt theo menu, hoặc dùng **Khởi động nhanh**.

Các trang của web: I2C (quét, kiểm tra ổn định), ToF (số đo, độ nhiễu, hiệu chuẩn), MPU (hiệu chuẩn,
góc thời gian thực, hệ số gyro), động cơ và encoder (lái tay, kiểm tra số xung/vòng), PID (đo đặc tính
động cơ, chạy vòng kín, test bậc thang), chạy thẳng, quay tại chỗ, và thông số (sao lưu JSON, xuất mã C).

Thông số và kết quả hiệu chuẩn được lưu trong trình duyệt. Mỗi lần kết nối, web gửi lại PID, bù trước
và hệ số gyro lên xe. Trang **Thông số** xuất được file JSON (mang sang máy khác) và đoạn mã C
(dán vào `firmware/lib/common/robot_config.h` khi viết code chính).

Giao thức lệnh và dữ liệu giữa web và xe được mô tả đầy đủ ở đầu `firmware/src/hwtest/hwtest.h`.

## Mô phỏng và lập quỹ đạo (Python)

Chạy từ thư mục gốc của repo. Lần đầu (sau khi clone) cần tạo môi trường Python `tools/.venv`, xem lệnh cài ở cuối mục này.

```sh
cd micromouse                                       # thư mục gốc của repo

tools/.venv/bin/python tools/planner/sim_gui.py   # mô phỏng tương tác (cửa sổ đồ hoạ)
tools/.venv/bin/python tools/planner/plan.py      # lập quỹ đạo, ghi kết quả vào tools/planner/out/
```

Hoặc kích hoạt môi trường một lần rồi gọi `python` ngắn gọn:

```sh
source tools/.venv/bin/activate
python tools/planner/sim_gui.py
deactivate                                        # thoát môi trường khi xong
```

Tạo môi trường (một lần sau khi clone, hoặc khi báo thiếu thư viện):

```sh
python3 -m venv tools/.venv
tools/.venv/bin/pip install numpy scipy matplotlib
```

Cách dùng mô phỏng, ý nghĩa các file kết quả và cách sửa bản đồ / thông số xe: xem `tools/planner/README.md`.

## Sửa cấu hình bằng CubeMX

Mở `firmware/firmware_micromouse.ioc`, chỉnh rồi bấm **Generate Code**. CubeMX giữ nguyên phần code nằm giữa các
cặp `USER CODE BEGIN / END` (lời gọi `hwtest_run()` trong `main.c` nằm trong đó) và không đụng tới
`src/hwtest/`, `lib/`. CubeMX sẽ tạo lại vài file CMake trong `firmware/`; PlatformIO không dùng chúng
và `.gitignore` đã bỏ qua.
