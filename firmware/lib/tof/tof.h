/* Dãy 5 cảm biến khoảng cách VL53L0X trên I2C1 (SCL PB8, SDA PB9).
 * Cả 5 cảm biến khởi động ở cùng địa chỉ 0x29, nên được bật lần lượt bằng chân XSHUT
 * và gán địa chỉ riêng 0x30..0x34 theo thứ tự L90, L45, M0, R45, R90. */
#ifndef TOF_H
#define TOF_H

#include <stdint.h>

#define TOF_COUNT     5
#define TOF_ADDR_BASE 0x30

enum { TOF_L90, TOF_L45, TOF_M0, TOF_R45, TOF_R90 };
extern const char *const TOF_NAME[TOF_COUNT];

/* Gọi sau khi khởi tạo xong mỗi cảm biến (để báo tiến độ ra ngoài). */
typedef void (*tof_init_cb)(int idx, uint8_t addr, int ok);

int tof_init_all(tof_init_cb cb); /* chặn ~0,5 s; trả về mặt nạ bit các cảm biến chạy được */
void tof_poll_one(void);          /* không chặn: mỗi lần chỉ hỏi 1 cảm biến, lần lượt vòng tròn */
int16_t tof_mm(int idx);          /* số đo mới nhất (mm), -1 = ngoài tầm / chưa có */
int tof_ready_mask(void);

#endif /* TOF_H */
