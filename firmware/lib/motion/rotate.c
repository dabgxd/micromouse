/* Quay tại chỗ:
 *   profile góc : giảm tốc theo góc CÒN LẠI THẬT (MPU) và giữ tốc độ bò tối thiểu tới khi tới đích,
 *                 nên xe quay liền một mạch, không "dừng rồi nhích"
 *   vòng ngoài  : dw = kt*sai_góc + ktd*(w_đặt - gz)  (độ/s), gyro đo góc thật kể cả khi bánh trượt
 *   vòng trong  : tốc độ mỗi bánh = (w_đặt + dw) * xung/độ, ngược chiều nhau; PID vị trí encoder từng bánh
 *   pha ổn định : hết profile, chỉnh tiếp tới khi sai < RT_TOL + 0,1° và xe đứng yên 80 ms (tối đa 800 ms)
 * Không có MPU (hoặc kt = ktd = 0): góc tính từ encoder và khoảng cách 2 bánh. */
#include <math.h>

#include "encoder.h"
#include "imu.h"
#include "main.h"
#include "motion_internal.h"
#include "pid.h"
#include "robot_config.h"
#include "tb6612.h"

#define RT_TOL     0.3f  /* độ: coi như tới đích */
#define RT_W_CREEP 25.0f /* độ/s: tốc độ bò tối thiểu ở cuối hành trình */
#define RT_LEAD    3.0f  /* độ: góc đặt được phép đi trước đích bấy nhiêu khi xe thật còn trễ */

static struct {
  int active, settling, gyro, sgn;
  float target, th, w, wmax, alpha, kt, ktd, cpd, yaw0, th_ref, th_meas, dw;
  float pos_ref[2];
  wheel_pid_t pid[2];
  int32_t start[2];
  uint32_t t0, prof_end, ok_since;
} rt;

void rotate_stop(void) { rt.active = 0; }
int rotate_active(void) { return rt.active; }

void motion_rotate_start(float deg, float wmax_dps, float alpha_dps2, float kt, float ktd, float counts_per_deg)
{
  motion_stop();
  if (deg == 0 || wmax_dps <= 0 || alpha_dps2 <= 0 || counts_per_deg <= 0) return;
  rt = (typeof(rt)){.active = 1, .gyro = imu_ok() && (kt != 0 || ktd != 0), .sgn = deg > 0 ? 1 : -1,
                    .target = fabsf(deg), .wmax = wmax_dps, .alpha = alpha_dps2, .kt = kt, .ktd = ktd,
                    .cpd = counts_per_deg, .yaw0 = imu_att()->yaw, .t0 = HAL_GetTick()};
  for (int s = 0; s < 2; s++) {
    rt.start[s] = encoder_count(s);
    wheel_pid_reset(&rt.pid[s], rt.start[s]);
  }
}

void rotate_step(float dt)
{
  if (!rt.active) return;
  const imu_att_t *a = imu_att();
  float dl = (float)(encoder_count(SIDE_L) - rt.start[SIDE_L]);
  float dr = (float)(encoder_count(SIDE_R) - rt.start[SIDE_R]);
  rt.th_meas = rt.gyro ? a->yaw - rt.yaw0 : (dr - dl) / 2 / rt.cpd;

  if (!rt.settling) {
    float rem_ref = rt.target - rt.th;
    float rem = rt.gyro ? fminf(rem_ref + RT_LEAD, rt.target - rt.sgn * rt.th_meas) : rem_ref;
    float w = trap_next_vel(rt.w, rt.wmax, rt.alpha, rem, dt);
    if (rt.gyro && rem > RT_TOL) w = fmaxf(w, RT_W_CREEP);
    rt.w = w;
    rt.th = fminf(rt.th + w * dt, rt.target + RT_LEAD);
    if (rem <= RT_TOL || (!rt.gyro && rt.th >= rt.target - 0.05f)) {
      if (!rt.gyro) rt.th = rt.target;
      rt.w = 0;
      rt.settling = 1;
      rt.prof_end = HAL_GetTick();
    }
  }
  float w_ref = rt.sgn * rt.w;
  rt.th_ref = rt.sgn * (rt.settling ? rt.target : rt.th);
  float e = rt.gyro && rt.settling ? rt.sgn * rt.target - rt.th_meas : rt.th_ref - rt.th_meas;

  float dw = 0;
  if (rt.gyro) {
    dw = rt.kt * e + rt.ktd * (w_ref - a->gz);
    float lim = 0.5f * rt.wmax + 30;
    dw = dw > lim ? lim : dw < -lim ? -lim : dw;
  }
  rt.dw = dw;
  float vdiff = (w_ref + dw) * rt.cpd; /* xung/s mỗi bánh */
  float vw[2] = {-vdiff, vdiff};

  for (int s = 0; s < 2; s++) {
    int32_t enc = encoder_count(s);
    rt.pos_ref[s] += vw[s] * dt;
    wheel_pid_measure(&rt.pid[s], enc, 1, dt);
    float err = rt.pos_ref[s] - (float)(enc - rt.start[s]);
    tb6612_set(s, wheel_pid_output(&rt.pid[s], err, vw[s], ff_breakaway(s, vw[s], rt.pid[s].vel_est), 0, dt));
  }

  if (rt.settling) {
    uint32_t now = HAL_GetTick();
    int ok = fabsf(e) < RT_TOL + 0.1f && fabsf(a->gz) < 3 && fabsf(rt.pid[SIDE_L].err) < 8 &&
             fabsf(rt.pid[SIDE_R].err) < 8;
    if (!ok) rt.ok_since = now;
    int done = rt.gyro ? (now - rt.ok_since > 80 || now - rt.prof_end > 800) : now - rt.prof_end > 300;
    if (done) {
      tb6612_brake(SIDE_L);
      tb6612_brake(SIDE_R);
      rt.active = 0;
      rotate_result_t r = {.err = rt.sgn * rt.target - rt.th_meas, .angle = rt.th_meas, .dl = (int32_t)dl,
                           .dr = (int32_t)dr, .ms = now - rt.t0, .settle_ms = now - rt.prof_end, .gyro = rt.gyro};
      if (motion_hooks->rotate_done) motion_hooks->rotate_done(&r);
    }
  }
}

void motion_rotate_tel(rotate_tel_t *t)
{
  t->th_ref = rt.th_ref;
  t->th = rt.th_meas;
  t->gz = imu_att()->gz;
  t->dw = rt.dw;
  t->dl = encoder_count(SIDE_L) - rt.start[SIDE_L];
  t->dr = encoder_count(SIDE_R) - rt.start[SIDE_R];
  for (int s = 0; s < 2; s++) {
    t->err[s] = rt.pid[s].err;
    t->u[s] = rt.pid[s].u;
  }
}
