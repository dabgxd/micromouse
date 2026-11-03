/* move: mỗi bánh đi đúng N xung (profile hình thang + bù trước + PID vị trí).
 * vel : giữ vận tốc từng bánh; vị trí đặt = tích phân vận tốc đặt (vận tốc đặt tiến dần theo gia tốc). */
#include <math.h>

#include "encoder.h"
#include "main.h"
#include "motion_internal.h"
#include "pid.h"
#include "robot_config.h"
#include "tb6612.h"

#define VEL_WDOG_MS   600 /* mất lệnh vel quá lâu (rớt Bluetooth) -> giảm tốc về 0 */
#define VEL_ERR_CLAMP 200 /* xung: động cơ bão hoà thì không dồn sai lệch, tránh "chạy bù" khi giảm tốc */

typedef struct {
  int active, dir, holding, velmode;
  float target, pos_ref, vel_ref, vmax, acc, max_err, vtarget; /* trong hệ quy chiếu đã quy về chiều dương */
  wheel_pid_t pid;
  int32_t start;
  uint32_t t0, hold_tick, limit_ms;
} move_t;

static move_t mv[2];
static uint32_t vel_cmd_tick;

void move_stop(void) { mv[SIDE_L].active = mv[SIDE_R].active = 0; }
int move_active(void) { return mv[SIDE_L].active || mv[SIDE_R].active; }
int vel_active(void) { return (mv[SIDE_L].active && mv[SIDE_L].velmode) || (mv[SIDE_R].active && mv[SIDE_R].velmode); }

void motion_move_start(int32_t counts_l, int32_t counts_r, float vmax, float acc)
{
  motion_stop();
  if (vmax <= 0 || acc <= 0) return;
  int32_t c[2] = {counts_l, counts_r};
  for (int s = 0; s < 2; s++) {
    if (!c[s]) continue;
    float T = fabsf((float)c[s]) / vmax + vmax / acc; /* thời gian profile (xấp xỉ) */
    int32_t enc = encoder_count(s);
    mv[s] = (move_t){.active = 1, .dir = c[s] > 0 ? 1 : -1, .target = fabsf((float)c[s]), .vmax = vmax, .acc = acc,
                     .start = enc, .t0 = HAL_GetTick(), .limit_ms = (uint32_t)(T * 2000) + 2000};
    wheel_pid_reset(&mv[s].pid, enc);
  }
}

void motion_vel_set(float v_l, float v_r, float acc)
{
  if (straight_active() || rotate_active()) motion_stop();
  if (acc <= 0) acc = 5000;
  float v[2] = {v_l, v_r};
  for (int s = 0; s < 2; s++) {
    if (!mv[s].active || !mv[s].velmode) { /* bắt đầu mới; đang chạy thì giữ nguyên trạng thái */
      int32_t enc = encoder_count(s);
      mv[s] = (move_t){.active = 1, .velmode = 1, .dir = 1, .start = enc, .t0 = HAL_GetTick()};
      wheel_pid_reset(&mv[s].pid, enc);
    }
    mv[s].vtarget = v[s];
    mv[s].acc = acc;
  }
  vel_cmd_tick = HAL_GetTick();
}

void move_step(float dt)
{
  for (int s = 0; s < 2; s++) {
    move_t *m = &mv[s];
    if (!m->active) continue;
    int32_t enc = encoder_count(s);
    float pos = (float)(enc - m->start) * m->dir;
    wheel_pid_measure(&m->pid, enc, (float)m->dir, dt);

    if (m->velmode) {
      if (HAL_GetTick() - vel_cmd_tick > VEL_WDOG_MS) m->vtarget = 0;
      float dv = m->vtarget - m->vel_ref, step = m->acc * dt;
      m->vel_ref += dv > step ? step : dv < -step ? -step : dv;
      m->pos_ref += m->vel_ref * dt;
      float e = m->pos_ref - pos;
      if (e > VEL_ERR_CLAMP) m->pos_ref = pos + VEL_ERR_CLAMP;
      if (e < -VEL_ERR_CLAMP) m->pos_ref = pos - VEL_ERR_CLAMP;
      if (m->vtarget == 0 && m->vel_ref == 0 && !m->holding) {
        m->holding = 1;
        m->hold_tick = HAL_GetTick();
      }
      if (m->vtarget != 0) m->holding = 0;
    } else if (!m->holding) {
      m->vel_ref = trap_next_vel(m->vel_ref, m->vmax, m->acc, m->target - m->pos_ref, dt);
      m->pos_ref += m->vel_ref * dt;
      if (m->pos_ref >= m->target - 0.5f) {
        m->pos_ref = m->target;
        m->vel_ref = 0;
        m->holding = 1;
        m->hold_tick = HAL_GetTick();
      }
    }

    float err = m->pos_ref - pos;
    if (fabsf(err) > m->max_err) m->max_err = fabsf(err);
    float u = wheel_pid_output(&m->pid, err, m->vel_ref, ff_kinetic(s, m->vel_ref), 0, dt);
    tb6612_set(s, u * m->dir);

    uint32_t now = HAL_GetTick();
    if (m->velmode) {
      if (m->holding && now - m->hold_tick > MOTION_HOLD_MS) {
        tb6612_brake(s);
        m->active = 0;
        if (!mv[!s].active && motion_hooks->vel_done) motion_hooks->vel_done();
      }
      continue;
    }
    int timeout = now - m->t0 > m->limit_ms;
    if ((m->holding && now - m->hold_tick > MOTION_HOLD_MS) || timeout) {
      tb6612_brake(s);
      m->active = 0;
      move_result_t r = {.side = s, .final_err = m->target - pos, .max_err = m->max_err,
                         .ms = (m->holding ? m->hold_tick : now) - m->t0, .timeout = timeout};
      if (motion_hooks->move_done) motion_hooks->move_done(&r);
    }
  }
}

void motion_move_tel(move_tel_t *t)
{
  for (int s = 0; s < 2; s++) {
    t->err[s] = mv[s].active ? mv[s].pos_ref - (float)(encoder_count(s) - mv[s].start) * mv[s].dir : 0;
    t->u[s] = mv[s].pid.u;
  }
}

void motion_vel_tel(vel_tel_t *t)
{
  for (int s = 0; s < 2; s++) {
    t->vref[s] = mv[s].vel_ref;
    t->u[s] = mv[s].pid.u;
    t->err[s] = mv[s].active ? mv[s].pos_ref - (float)(encoder_count(s) - mv[s].start) : 0;
  }
}
