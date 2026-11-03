/* Chạy thẳng: một profile hình thang cho tâm xe; hai bánh chạy lệch nhau dv để giữ hướng
 *   dv = kh * (hướng đặt - yaw) - kg * gz    (xung/s);  bánh trái v - dv, bánh phải v + dv
 * Mỗi bánh: PID vị trí + bù trước (có bù ma sát tĩnh) + ghép chéo kc*(sai_trái - sai_phải)
 * để hai bánh luôn đi đều. Một bánh tụt quá ST_LAG_MAX xung thì profile đứng chờ. */
#include <math.h>

#include "encoder.h"
#include "imu.h"
#include "main.h"
#include "motion_internal.h"
#include "pid.h"
#include "robot_config.h"
#include "tb6612.h"

#define ST_LAG_MAX 60 /* xung */

static struct {
  int active, holding, gyro;
  trap_t prof; /* prof.pos = quãng đường đặt của tâm xe, prof.vel = vận tốc đặt */
  float kh, kg, kc, yaw_set, dv, yaw_err, max_yaw_err;
  float pos_ref[2];
  wheel_pid_t pid[2];
  int32_t start[2];
  uint32_t t0, hold_tick, limit_ms;
} st;

void straight_stop(void) { st.active = 0; }
int straight_active(void) { return st.active; }

void motion_straight_start(float counts, float vmax, float acc, float kh, float kg, float yaw_off_deg, float kc)
{
  motion_stop();
  if (counts <= 0 || vmax <= 0 || acc <= 0) return;
  float T = counts / vmax + vmax / acc;
  st = (typeof(st)){.active = 1, .gyro = imu_ok(), .prof = {.target = counts, .vmax = vmax, .acc = acc},
                    .kh = kh, .kg = kg, .kc = kc, .yaw_set = imu_att()->yaw + yaw_off_deg, .t0 = HAL_GetTick(),
                    .limit_ms = (uint32_t)(T * 2000) + 2000};
  for (int s = 0; s < 2; s++) {
    st.start[s] = encoder_count(s);
    wheel_pid_reset(&st.pid[s], st.start[s]);
  }
}

void straight_step(float dt)
{
  if (!st.active) return;
  int lagging = fabsf(st.pid[SIDE_L].err) > ST_LAG_MAX || fabsf(st.pid[SIDE_R].err) > ST_LAG_MAX;
  if (!st.holding && !lagging && trap_step(&st.prof, dt, 0.5f)) {
    st.holding = 1;
    st.hold_tick = HAL_GetTick();
  }

  const imu_att_t *a = imu_att();
  st.yaw_err = st.yaw_set - a->yaw;
  if (fabsf(st.yaw_err) > st.max_yaw_err) st.max_yaw_err = fabsf(st.yaw_err);
  float dv = 0;
  if (st.gyro && !st.holding) {
    dv = st.kh * st.yaw_err - st.kg * a->gz;
    float lim = 0.3f * st.prof.vmax + 100; /* không để bánh nào đổi chiều */
    dv = dv > lim ? lim : dv < -lim ? -lim : dv;
  }
  st.dv = dv;

  float vw[2] = {st.prof.vel - dv, st.prof.vel + dv};
  if (lagging) vw[SIDE_L] = vw[SIDE_R] = 0; /* đứng chờ bánh tụt lại */
  float e[2];
  for (int s = 0; s < 2; s++) {
    int32_t enc = encoder_count(s);
    st.pos_ref[s] += vw[s] * dt;
    wheel_pid_measure(&st.pid[s], enc, 1, dt);
    e[s] = st.pos_ref[s] - (float)(enc - st.start[s]);
  }
  float cross = st.kc * (e[SIDE_L] - e[SIDE_R]); /* bánh tụt lại được đẩy thêm, bánh vượt lên bị hãm bớt */
  for (int s = 0; s < 2; s++) {
    float ff = ff_breakaway(s, vw[s], st.pid[s].vel_est);
    tb6612_set(s, wheel_pid_output(&st.pid[s], e[s], vw[s], ff, s == SIDE_L ? cross : -cross, dt));
  }

  uint32_t now = HAL_GetTick();
  int timeout = now - st.t0 > st.limit_ms;
  if ((st.holding && now - st.hold_tick > MOTION_HOLD_MS) || timeout) {
    tb6612_brake(SIDE_L);
    tb6612_brake(SIDE_R);
    st.active = 0;
    straight_result_t r = {.dl = encoder_count(SIDE_L) - st.start[SIDE_L], .dr = encoder_count(SIDE_R) - st.start[SIDE_R],
                           .yaw_err = st.yaw_err, .max_yaw_err = st.max_yaw_err,
                           .ms = (st.holding ? st.hold_tick : now) - st.t0, .gyro = st.gyro};
    if (motion_hooks->straight_done) motion_hooks->straight_done(&r);
  }
}

void motion_straight_tel(straight_tel_t *t)
{
  t->s_ref = st.prof.pos;
  t->vel_ref = st.prof.vel;
  t->yaw_err = st.yaw_err;
  t->dv = st.dv;
  t->dl = encoder_count(SIDE_L) - st.start[SIDE_L];
  t->dr = encoder_count(SIDE_R) - st.start[SIDE_R];
  for (int s = 0; s < 2; s++) {
    t->err[s] = st.pid[s].err;
    t->u[s] = st.pid[s].u;
  }
}
