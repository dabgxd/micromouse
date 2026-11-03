/* Xử lý lệnh từ web (xem bảng giao thức trong hwtest.h). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "encoder.h"
#include "hwtest_internal.h"
#include "imu.h"
#include "link.h"
#include "main.h"
#include "motion.h"
#include "mpu6050.h"
#include "pid.h"
#include "robot_config.h"
#include "tb6612.h"
#include "tof.h"

extern I2C_HandleTypeDef hi2c1, hi2c2;

/* ========================= I2C ========================= */
static void cmd_scan(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDR << 1, 2, 5) != HAL_OK) imu_bus_recover();
  I2C_HandleTypeDef *bus[2] = {&hi2c1, &hi2c2};
  for (int b = 0; b < 2; b++) {
    char s[120];
    int n = snprintf(s, sizeof s, "I2C,%d", b + 1);
    for (uint8_t a = 1; a < 0x78 && n < (int)sizeof s - 4; a++)
      if (HAL_I2C_IsDeviceReady(bus[b], a << 1, 1, 2) == HAL_OK) n += snprintf(s + n, sizeof s - n, ",%02X", a);
    link_printf("%s\n", s);
  }
}

/* ========================= ToF, MPU ========================= */
static void tof_report(int idx, uint8_t addr, int ok)
{
  link_printf("TOFI,%d,%02X,%d\n", idx, addr, ok);
  link_flush();
  HAL_Delay(15); /* chờ DMA gửi xong dòng trước khi khởi tạo cảm biến tiếp theo */
}

static void cal_progress(int pct) { link_printf("CALP,%d\n", pct); }

static void cmd_mpu_cal(void)
{
  if (!imu_ok()) {
    link_printf("LOG,Chua khoi tao MPU\n");
    return;
  }
  app_stop_all();
  float noise, b[3];
  if (!imu_calibrate(3000, cal_progress, &noise)) {
    link_printf("LOG,MPU khong tra du lieu\n");
    return;
  }
  imu_get_bias(b);
  link_printf("CALD,%.3f,%.3f,%.3f,%.4f\n", b[0], b[1], b[2], noise);
}

/* ========================= đo đặc tính động cơ ========================= */
/* Chờ ms mili giây (vẫn đếm encoder, vẫn gửi dữ liệu). Trả về 1 nếu web gửi "stop". */
static int wait_abort(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  char line[64];
  while (HAL_GetTick() - t0 < ms) {
    encoder_update();
    link_flush();
    if (link_read_line(line, sizeof line) && !strncmp(line, "stop", 4)) return 1;
  }
  return 0;
}

/* Kê bánh lên. 1) tăng PWM dần tới khi bánh quay > 30 xung/s -> PWM khởi động;
 * 2) tốc độ ổn định ở 7 mức PWM -> web tính mô hình bù trước. */
static void cmd_ident(void)
{
  app_stop_all();
  float dead[2] = {0, 0};
  for (int pct = 1; pct <= 40 && (!dead[SIDE_L] || !dead[SIDE_R]); pct++) {
    for (int s = 0; s < 2; s++) tb6612_set(s, dead[s] ? 0 : pct);
    int32_t a[2] = {encoder_count(SIDE_L), encoder_count(SIDE_R)};
    if (wait_abort(200)) goto abort;
    for (int s = 0; s < 2; s++)
      if (!dead[s] && labs(encoder_count(s) - a[s]) * 5 > 30) dead[s] = pct;
  }
  app_stop_all();
  link_printf("IDS,%.0f,%.0f\n", dead[SIDE_L], dead[SIDE_R]);
  if (wait_abort(500)) goto abort;

  static const int steps[] = {15, 25, 40, 55, 70, 85, 100};
  for (unsigned k = 0; k < sizeof steps / sizeof steps[0]; k++) {
    tb6612_set(SIDE_L, steps[k]);
    tb6612_set(SIDE_R, steps[k]);
    if (wait_abort(450)) goto abort;
    int32_t a[2] = {encoder_count(SIDE_L), encoder_count(SIDE_R)};
    if (wait_abort(300)) goto abort;
    link_printf("IDN,%d,%.0f,%.0f\n", steps[k], (encoder_count(SIDE_L) - a[SIDE_L]) / 0.3f,
                (encoder_count(SIDE_R) - a[SIDE_R]) / 0.3f);
  }
  app_stop_all();
  link_printf("IDD\n");
  return;
abort:
  app_stop_all();
  link_printf("LOG,Da huy do dac tinh\n");
}

/* ========================= bộ phân tích lệnh ========================= */
static float argf(const char *s, float def) { return s ? strtof(s, NULL) : def; }

void commands_dispatch(char *line)
{
  char *cmd = strtok(line, " ");
  char *a[7];
  for (int i = 0; i < 7; i++) a[i] = strtok(NULL, " ");
  if (!cmd) return;

  if (!strcmp(cmd, "ping")) {
    link_printf("PONG\n");
  } else if (!strcmp(cmd, "scan")) {
    cmd_scan();
  } else if (!strcmp(cmd, "stop")) {
    app_stop_all();
  } else if (!strcmp(cmd, "tof") && a[0]) {
    if (!strcmp(a[0], "init")) {
      tof_init_all(tof_report);
    } else if (!strcmp(a[0], "on")) {
      if (!tof_ready_mask()) tof_init_all(tof_report);
      tel_tof = 1;
    } else {
      tel_tof = 0;
    }
  } else if (!strcmp(cmd, "mpu") && a[0]) {
    if (!strcmp(a[0], "init")) link_printf("MPUI,%02X\n", imu_init());
    else if (!strcmp(a[0], "cal")) cmd_mpu_cal();
    else if (!strcmp(a[0], "zero")) imu_zero_yaw();
    else if (!strcmp(a[0], "scale") && a[1]) imu_set_scale(argf(a[1], 1));
    else if (!strcmp(a[0], "on")) tel_imu = imu_ok();
    else tel_imu = 0;
  } else if (!strcmp(cmd, "m") && a[1]) {
    app_manual(argf(a[0], 0), argf(a[1], 0));
  } else if (!strcmp(cmd, "enc") && a[0] && !strcmp(a[0], "zero")) {
    encoder_zero();
  } else if (!strcmp(cmd, "go") && a[2]) {
    app_go_start(atoi(a[0]), labs(atol(a[1])), argf(a[2], 0));
  } else if (!strcmp(cmd, "mv") && a[2]) {
    float vmax = argf(a[2], 0), acc = argf(a[3], 0);
    if (vmax > 0 && acc > 0) {
      app_stop_all();
      motion_move_start(atol(a[0]), atol(a[1]), vmax, acc);
    }
  } else if (!strcmp(cmd, "vel") && a[2]) {
    app_cancel_manual();
    motion_vel_set(argf(a[0], 0), argf(a[1], 0), argf(a[2], 0));
  } else if (!strcmp(cmd, "st") && a[2]) {
    app_stop_all();
    motion_straight_start(argf(a[0], 0), argf(a[1], 0), argf(a[2], 0), argf(a[3], 30), argf(a[4], 3), argf(a[5], 0),
                          argf(a[6], 0.2f));
  } else if (!strcmp(cmd, "rot") && a[2]) {
    app_stop_all();
    motion_rotate_start(argf(a[0], 0), argf(a[1], 0), argf(a[2], 0), argf(a[3], 8), argf(a[4], 0.5f),
                        argf(a[5], ENC_CPR_WHEEL / WHEEL_CIRC_MM * TRACK_MM / 2 * 3.14159265f / 180));
  } else if (!strcmp(cmd, "pid") && a[2]) {
    pid_gains = (pid_gains_t){.kp = argf(a[0], 0), .ki = argf(a[1], 0), .kd = argf(a[2], 0)};
    link_printf("LOG,PID kp=%.3f ki=%.3f kd=%.4f\n", pid_gains.kp, pid_gains.ki, pid_gains.kd);
  } else if (!strcmp(cmd, "ff") && a[2]) {
    ff_model.dead[SIDE_L] = argf(a[0], 0);
    ff_model.kv[SIDE_L] = fmaxf(argf(a[1], 1), 1);
    ff_model.dead[SIDE_R] = argf(a[2], 0);
    ff_model.kv[SIDE_R] = fmaxf(argf(a[3], ff_model.kv[SIDE_L]), 1);
    for (int s = 0; s < 2; s++) ff_model.stat[s] = ff_model.dead[s] * 1.3f + 1; /* mặc định nếu chưa đo */
    if (a[4]) ff_model.stat[SIDE_L] = argf(a[4], 0);
    if (a[5]) ff_model.stat[SIDE_R] = argf(a[5], 0);
    link_printf("LOG,FF trai %.1f%%+v/%.1f, phai %.1f%%+v/%.1f\n", ff_model.dead[SIDE_L], ff_model.kv[SIDE_L],
                ff_model.dead[SIDE_R], ff_model.kv[SIDE_R]);
  } else if (!strcmp(cmd, "ident")) {
    cmd_ident();
  } else {
    link_printf("LOG,Lenh la: %s\n", cmd);
  }
}
