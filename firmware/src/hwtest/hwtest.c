/* Vòng lặp chính của firmware test: nhịp 1 kHz cho điều khiển + MPU, telemetry 25 Hz, xử lý lệnh từ web. */
#include "hwtest.h"

#include <stdlib.h>

#include "encoder.h"
#include "hwtest_internal.h"
#include "imu.h"
#include "link.h"
#include "main.h"
#include "motion.h"
#include "robot_config.h"
#include "tb6612.h"
#include "timebase.h"
#include "tof.h"

#define TELEMETRY_MS   40   /* 25 Hz mặc định; web giảm xuống khi dùng BLE (lệnh "tel") */
#define MANUAL_WDOG_MS 500  /* lái tay: không nhận lệnh "m" quá lâu (rớt Bluetooth) -> dừng */
#define GO_TIMEOUT_MS  8000 /* chạy hở: encoder không đếm (đứt dây) thì vẫn dừng */
#define STILL_ENC_MS   300  /* encoder đứng yên bấy lâu mới cho MPU tự bù trôi */

int tel_tof, tel_imu, tel_map;
uint32_t tel_ms = TELEMETRY_MS;

static uint32_t manual_tick, enc_move_tick;
static struct {
  int active;
  int32_t start, target;
  uint32_t start_tick, done_tick;
} go[2];

/* ========================= lái tay, chạy hở ========================= */
void app_cancel_manual(void)
{
  manual_tick = 0;
  go[SIDE_L].active = go[SIDE_R].active = 0;
}

void app_stop_all(void)
{
  app_cancel_manual();
  motion_stop();
}

void app_manual(float pct_l, float pct_r)
{
  if (motion_busy()) motion_stop();
  go[SIDE_L].active = go[SIDE_R].active = 0;
  tb6612_set(SIDE_L, pct_l);
  tb6612_set(SIDE_R, pct_r);
  manual_tick = HAL_GetTick();
}

void app_go_start(int which, int32_t counts, float pct)
{
  app_stop_all();
  for (int s = 0; s < 2 && counts > 0; s++) {
    if (which != 2 && which != s) continue;
    go[s] = (typeof(go[s])){.active = 1, .start = encoder_count(s), .target = counts, .start_tick = HAL_GetTick()};
  }
  for (int s = 0; s < 2; s++) /* bật cùng lúc để hai bánh xuất phát đồng thời */
    if (go[s].active) tb6612_set(s, pct);
}

/* Đủ xung thì phanh, chờ 400 ms cho bánh đứng hẳn rồi báo số xung thật. */
static void go_update(void)
{
  for (int s = 0; s < 2; s++) {
    if (!go[s].active) continue;
    int32_t d = labs(encoder_count(s) - go[s].start);
    int timeout = HAL_GetTick() - go[s].start_tick > GO_TIMEOUT_MS;
    if (!go[s].done_tick && (d >= go[s].target || timeout)) {
      tb6612_brake(s);
      go[s].done_tick = HAL_GetTick();
    }
    if (go[s].done_tick && HAL_GetTick() - go[s].done_tick > 400) {
      tb6612_set(s, 0);
      link_printf("GOD,%d,%ld,%lu,%d\n", s, (long)(encoder_count(s) - go[s].start),
                  (unsigned long)(go[s].done_tick - go[s].start_tick), timeout);
      go[s].active = 0;
    }
  }
}

/* Xe có đang (hoặc vừa) chuyển động không: quyết định MPU có được tự bù trôi. */
static int is_moving(void)
{
  return manual_tick || go[SIDE_L].active || go[SIDE_R].active || motion_busy() ||
         HAL_GetTick() - enc_move_tick <= STILL_ENC_MS;
}

/* ========================= báo kết thúc chuyển động ========================= */
static void on_move_done(const move_result_t *r)
{
  link_printf("MVD,%d,%.1f,%.1f,%lu,%d\n", r->side, r->final_err, r->max_err, (unsigned long)r->ms, r->timeout);
}

static void on_vel_done(void) { link_printf("VLD\n"); }

static void on_straight_done(const straight_result_t *r)
{
  link_printf("STD,%ld,%ld,%.2f,%.2f,%lu,%d\n", (long)r->dl, (long)r->dr, r->yaw_err, r->max_yaw_err,
              (unsigned long)r->ms, r->gyro);
}

static void on_rotate_done(const rotate_result_t *r)
{
  link_printf("RTD,%.2f,%.2f,%ld,%ld,%lu,%d,%lu\n", r->err, r->angle, (long)r->dl, (long)r->dr, (unsigned long)r->ms,
              r->gyro, (unsigned long)r->settle_ms);
}

static const motion_hooks_t hooks = {
    .move_done = on_move_done, .vel_done = on_vel_done, .straight_done = on_straight_done, .rotate_done = on_rotate_done};

/* ========================= telemetry ========================= */
static void telemetry(uint32_t dt_ms)
{
  static int32_t prev[2];
  int32_t c[2] = {encoder_count(SIDE_L), encoder_count(SIDE_R)};
  float k = 1000.0f / dt_ms;
  float spd[2] = {(c[SIDE_L] - prev[SIDE_L]) * k, (c[SIDE_R] - prev[SIDE_R]) * k};
  prev[SIDE_L] = c[SIDE_L];
  prev[SIDE_R] = c[SIDE_R];
  link_printf("ENC,%ld,%ld,%.0f,%.0f\n", (long)c[SIDE_L], (long)c[SIDE_R], spd[SIDE_L], spd[SIDE_R]);

  if (tel_map) { /* quét bản đồ: encoder + yaw + 5 ToF trong cùng một dòng, cùng một thời điểm */
    link_printf("MAP,%lu,%ld,%ld,%.2f,%d,%d,%d,%d,%d\n", (unsigned long)HAL_GetTick(), (long)c[SIDE_L],
                (long)c[SIDE_R], imu_ok() ? imu_att()->yaw : 0.0f, tof_mm(TOF_L90), tof_mm(TOF_L45),
                tof_mm(TOF_M0), tof_mm(TOF_R45), tof_mm(TOF_R90));
  }
  if (tel_tof)
    link_printf("TOF,%d,%d,%d,%d,%d\n", tof_mm(TOF_L90), tof_mm(TOF_L45), tof_mm(TOF_M0), tof_mm(TOF_R45),
                tof_mm(TOF_R90));
  if (tel_imu) {
    const imu_att_t *a = imu_att();
    float b[3];
    imu_get_bias(b);
    link_printf("IMU,%.2f,%.2f,%.2f,%.2f,%d,%lu,%d,%.3f\n", a->yaw, a->roll, a->pitch, a->gz, imu_live(),
                (unsigned long)imu_recover_count(), imu_still(), b[2]);
  }

  switch (motion_mode()) {
  case MOTION_ROTATE: {
    rotate_tel_t t;
    motion_rotate_tel(&t);
    link_printf("RTT,%.2f,%.2f,%.1f,%.1f,%ld,%ld,%.0f,%.0f,%.1f,%.1f\n", t.th_ref, t.th, t.gz, t.dw, (long)t.dl,
                (long)t.dr, t.u[SIDE_L], t.u[SIDE_R], t.err[SIDE_L], t.err[SIDE_R]);
    break;
  }
  case MOTION_STRAIGHT: {
    straight_tel_t t;
    motion_straight_tel(&t);
    link_printf("STT,%.0f,%ld,%ld,%.2f,%.0f,%.0f,%.0f,%.0f,%.1f,%.1f\n", t.s_ref, (long)t.dl, (long)t.dr, t.yaw_err,
                t.dv, t.u[SIDE_L], t.u[SIDE_R], t.vel_ref, t.err[SIDE_L], t.err[SIDE_R]);
    break;
  }
  case MOTION_VEL: {
    vel_tel_t t;
    motion_vel_tel(&t);
    link_printf("VLT,%.0f,%.0f,%.1f,%.1f,%.0f,%.0f,%.1f,%.1f\n", t.vref[SIDE_L], spd[SIDE_L], t.u[SIDE_L],
                t.err[SIDE_L], t.vref[SIDE_R], spd[SIDE_R], t.u[SIDE_R], t.err[SIDE_R]);
    break;
  }
  case MOTION_MOVE: {
    move_tel_t t;
    motion_move_tel(&t);
    link_printf("MVT,%.1f,%.1f,%.0f,%.0f\n", t.err[SIDE_L], t.err[SIDE_R], t.u[SIDE_L], t.u[SIDE_R]);
    break;
  }
  default:
    break;
  }
}

/* ========================= vòng lặp chính ========================= */
void hwtest_run(void)
{
  timebase_init();
  link_init();
  encoder_init();
  tb6612_init();
  motion_init(&hooks);
  app_stop_all();
  link_printf("HELLO\n");

  char line[64];
  uint32_t last_tel = 0, blink = 0, next_1k = cyc_now();
  int32_t enc_moved[2] = {0, 0};
  while (1) {
    encoder_update();
    go_update();
    if (manual_tick && HAL_GetTick() - manual_tick > MANUAL_WDOG_MS) app_stop_all();

    if ((int32_t)(cyc_now() - next_1k) >= 0) { /* nhịp 1 kHz */
      next_1k += CYC_PER_MS;
      if ((int32_t)(cyc_now() - next_1k) > 0) next_1k = cyc_now() + CYC_PER_MS; /* bị trễ: bắt nhịp lại */
      for (int s = 0; s < 2; s++)
        if (labs(encoder_count(s) - enc_moved[s]) > 2) {
          enc_moved[SIDE_L] = encoder_count(SIDE_L);
          enc_moved[SIDE_R] = encoder_count(SIDE_R);
          enc_move_tick = HAL_GetTick();
        }
      motion_step(CTRL_DT_S);
      imu_step(is_moving());
    }

    if (tof_ready_mask()) tof_poll_one();
    if (link_read_line(line, sizeof line)) commands_dispatch(line);

    uint32_t now = HAL_GetTick();
    if (now - last_tel >= tel_ms) {
      telemetry(now - last_tel);
      last_tel = now;
    }
    link_flush();
    if (now - blink >= 500) { /* LED PC13 nháy = firmware test đang chạy */
      blink = now;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
}
