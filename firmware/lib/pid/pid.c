#include "pid.h"

#include <math.h>

/* Giá trị mặc định khi chưa chỉnh / chưa đo; web gửi giá trị thật mỗi lần kết nối. */
pid_gains_t pid_gains = {.kp = 0.4f, .ki = 3.0f, .kd = 0.01f};
ff_model_t ff_model = {.dead = {6, 6}, .kv = {55, 55}, .stat = {8, 8}};

#define I_LIMIT_PCT 30.0f /* phần tích phân đóng góp tối đa ±30 % PWM */

float ff_kinetic(int side, float v)
{
  if (fabsf(v) < 1) return 0;
  return copysignf(ff_model.dead[side] + fabsf(v) / ff_model.kv[side], v);
}

float ff_breakaway(int side, float v, float v_meas)
{
  float u = ff_kinetic(side, v);
  if (fabsf(v) >= 1 && fabsf(v_meas) < 40 && fabsf(u) < ff_model.stat[side]) u = copysignf(ff_model.stat[side], v);
  return u;
}

void wheel_pid_reset(wheel_pid_t *w, int32_t enc_now) { *w = (wheel_pid_t){.last = enc_now}; }

void wheel_pid_measure(wheel_pid_t *w, int32_t enc_now, float sign, float dt)
{
  float v_meas = (float)(enc_now - w->last) * sign / dt;
  w->last = enc_now;
  w->vel_est += dt / (0.01f + dt) * (v_meas - w->vel_est);
}

float wheel_pid_output(wheel_pid_t *w, float err, float vel_ref, float ff_pct, float extra, float dt)
{
  w->err = err;
  w->integ += err * dt;
  float ilim = I_LIMIT_PCT / fmaxf(pid_gains.ki, 1e-3f);
  if (w->integ > ilim) w->integ = ilim;
  if (w->integ < -ilim) w->integ = -ilim;
  float u = ff_pct + pid_gains.kp * err + pid_gains.ki * w->integ + pid_gains.kd * (vel_ref - w->vel_est) + extra;
  w->u = u > 100 ? 100 : u < -100 ? -100 : u;
  return w->u;
}

float trap_next_vel(float vel, float vmax, float acc, float rem, float dt)
{
  return fminf(fminf(vel + acc * dt, vmax), sqrtf(2 * acc * fmaxf(rem, 0)));
}

int trap_step(trap_t *p, float dt, float eps)
{
  p->vel = trap_next_vel(p->vel, p->vmax, p->acc, p->target - p->pos, dt);
  p->pos += p->vel * dt;
  if (p->pos >= p->target - eps) {
    p->pos = p->target;
    p->vel = 0;
    return 1;
  }
  return 0;
}
