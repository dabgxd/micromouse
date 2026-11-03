/* Dùng chung giữa hwtest.c (vòng lặp, telemetry) và commands.c (xử lý lệnh). */
#ifndef HWTEST_INTERNAL_H
#define HWTEST_INTERNAL_H

#include <stdint.h>

extern int tel_tof, tel_imu; /* 1 = đang gửi telemetry TOF / IMU */

void app_stop_all(void);      /* dừng mọi thứ: chuyển động kín vòng, chạy hở, lái tay */
void app_cancel_manual(void); /* huỷ lái tay và chạy hở (không đụng chuyển động kín vòng) */
void app_manual(float pct_l, float pct_r);           /* lái tay, tự dừng nếu không được gửi lại trong 500 ms */
void app_go_start(int which, int32_t counts, float pct); /* chạy hở N xung rồi phanh; which: 0 L, 1 R, 2 cả hai */

void commands_dispatch(char *line);

#endif /* HWTEST_INTERNAL_H */
