/* Dùng riêng bên trong thư viện motion. */
#ifndef MOTION_INTERNAL_H
#define MOTION_INTERNAL_H

#include "motion.h"

#define MOTION_HOLD_MS 300 /* giữ vị trí sau khi profile kết thúc, rồi mới phanh */

extern const motion_hooks_t *motion_hooks;

void move_step(float dt);
void move_stop(void);
int move_active(void); /* 1 = move hoặc vel đang chạy */
int vel_active(void);

void straight_step(float dt);
void straight_stop(void);
int straight_active(void);

void rotate_step(float dt);
void rotate_stop(void);
int rotate_active(void);

#endif /* MOTION_INTERNAL_H */
