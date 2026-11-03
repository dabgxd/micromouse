/* Điều khiển chuyển động kín vòng của xe (chạy ở nhịp 1 kHz), kết hợp encoder + MPU + PID:
 *   move     : mỗi bánh đi đúng N xung theo profile hình thang (kiểm tra số xung/vòng)
 *   vel      : giữ vận tốc từng bánh liên tục (kiểm tra PID ở tốc độ cao); phải gọi lại mỗi < 600 ms
 *   straight : chạy thẳng N xung, giữ hướng bằng gyro (dv = kh*sai_hướng - kg*gz) + ghép chéo 2 bánh
 *   rotate   : quay tại chỗ N độ; vòng ngoài gyro sửa góc, vòng trong PID vị trí encoder
 * Mọi hàm *_start() tự huỷ chuyển động đang chạy. Kết thúc thì phanh và gọi hook tương ứng.
 * Quy ước: xung dương = bánh chạy tiến; góc dương = quay trái (ngược chiều kim đồng hồ). */
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

typedef enum { MOTION_IDLE = 0, MOTION_MOVE, MOTION_VEL, MOTION_STRAIGHT, MOTION_ROTATE } motion_mode_t;

typedef struct {
  int side;
  float final_err, max_err; /* xung */
  uint32_t ms;
  int timeout;
} move_result_t;

typedef struct {
  int32_t dl, dr;                /* xung mỗi bánh đã đi */
  float yaw_err, max_yaw_err;    /* độ */
  uint32_t ms;
  int gyro;                      /* 1 = có dùng MPU giữ hướng */
} straight_result_t;

typedef struct {
  float err, angle;              /* độ: sai còn lại, góc đã quay (đo bằng MPU, hoặc encoder nếu không có MPU) */
  int32_t dl, dr;
  uint32_t ms, settle_ms;
  int gyro;
} rotate_result_t;

typedef struct {
  void (*move_done)(const move_result_t *r);
  void (*vel_done)(void);
  void (*straight_done)(const straight_result_t *r);
  void (*rotate_done)(const rotate_result_t *r);
} motion_hooks_t;

void motion_init(const motion_hooks_t *hooks); /* hooks có thể NULL */
void motion_step(float dt);                    /* gọi mỗi 1 ms, sau encoder_update() */
void motion_stop(void);                        /* huỷ mọi chuyển động, thả trôi động cơ */
int motion_busy(void);
motion_mode_t motion_mode(void);

void motion_move_start(int32_t counts_l, int32_t counts_r, float vmax, float acc); /* xung, xung/s, xung/s² */
void motion_vel_set(float v_l, float v_r, float acc);                              /* xung/s, xung/s² */
void motion_straight_start(float counts, float vmax, float acc, float kh, float kg, float yaw_off_deg, float kc);
void motion_rotate_start(float deg, float wmax_dps, float alpha_dps2, float kt, float ktd, float counts_per_deg);

/* ---- Số liệu thời gian thực (telemetry) ---- */
typedef struct { float err[2], u[2]; } move_tel_t;
typedef struct { float vref[2], u[2], err[2]; } vel_tel_t;
typedef struct { float s_ref, vel_ref, yaw_err, dv, err[2], u[2]; int32_t dl, dr; } straight_tel_t;
typedef struct { float th_ref, th, gz, dw, err[2], u[2]; int32_t dl, dr; } rotate_tel_t;

void motion_move_tel(move_tel_t *t);
void motion_vel_tel(vel_tel_t *t);
void motion_straight_tel(straight_tel_t *t);
void motion_rotate_tel(rotate_tel_t *t);

#endif /* MOTION_H */
