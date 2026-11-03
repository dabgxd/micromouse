/* Khối điều khiển bánh xe dùng chung:
 *   1. Mô hình bù trước (feedforward) của động cơ: PWM = dead + v / kv, cộng bù ma sát tĩnh khi bánh đứng yên.
 *   2. PID bám vị trí cho từng bánh: u = ff + kp*e + ki*∫e + kd*(v_đặt - v_đo) + extra, kẹp ±100 %.
 *   3. Profile vận tốc hình thang: tăng tốc, chạy đều, giảm tốc về 0 đúng đích.
 * Đơn vị: vị trí = xung encoder, vận tốc = xung/s, u = % PWM. */
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  float kp; /* %/xung */
  float ki; /* %/(xung.s) */
  float kd; /* %/(xung/s) */
} pid_gains_t;

typedef struct {
  float dead[2]; /* PWM (%) tối thiểu để bánh đang quay tiếp tục quay */
  float kv[2];   /* xung/s tăng thêm cho mỗi 1 % PWM trên mức dead */
  float stat[2]; /* PWM (%) để bánh đang ĐỨNG YÊN bắt đầu quay (ma sát tĩnh) */
} ff_model_t;

extern pid_gains_t pid_gains; /* chỉnh từ web (lệnh "pid") */
extern ff_model_t ff_model;   /* đo bằng web: "Đo đặc tính động cơ" (lệnh "ident", "ff") */

float ff_kinetic(int side, float v);                 /* PWM (%) để bánh chạy v xung/s */
float ff_breakaway(int side, float v, float v_meas); /* như trên + bù ma sát tĩnh khi bánh gần đứng yên */

/* ---- PID vị trí cho 1 bánh ---- */
typedef struct {
  float integ;   /* ∫sai lệch */
  float vel_est; /* vận tốc đo đã lọc (xung/s) */
  float err;     /* sai lệch vị trí gần nhất (xung) */
  float u;       /* đầu ra gần nhất (%) */
  int32_t last;  /* số xung ở lần đo trước */
} wheel_pid_t;

void wheel_pid_reset(wheel_pid_t *w, int32_t enc_now);
/* Cập nhật vận tốc lọc (tau 10 ms). sign = -1 để đo trong hệ quy chiếu đảo chiều. */
void wheel_pid_measure(wheel_pid_t *w, int32_t enc_now, float sign, float dt);
/* Tích phân + tính đầu ra. ff_pct: phần bù trước đã tính; extra: số hạng cộng thêm (ví dụ ghép chéo 2 bánh). */
float wheel_pid_output(wheel_pid_t *w, float err, float vel_ref, float ff_pct, float extra, float dt);

/* ---- Profile hình thang (trên độ lớn, từ 0 tới target) ---- */
typedef struct {
  float target, pos, vel, vmax, acc;
} trap_t;

/* Vận tốc kế tiếp: tăng theo gia tốc, không quá vmax, và đủ thấp để dừng kịp trong quãng rem. */
float trap_next_vel(float vel, float vmax, float acc, float rem, float dt);
/* Tiến 1 bước. Trả về 1 khi tới đích (pos = target, vel = 0); eps = sai số coi như tới. */
int trap_step(trap_t *p, float dt, float eps);

#endif /* PID_H */
