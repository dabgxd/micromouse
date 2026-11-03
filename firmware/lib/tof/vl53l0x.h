/* Driver VL53L0X tối giản (chuyển từ thư viện Pololu sang C + HAL). */
#ifndef VL53L0X_H
#define VL53L0X_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define VL53L0X_DEFAULT_ADDR 0x29

typedef struct {
  I2C_HandleTypeDef *i2c;
  uint8_t addr;          /* địa chỉ 7 bit */
  uint8_t stop_variable;
  uint32_t budget_us;
  bool ok;
} vl53l0x_t;

/* Đổi địa chỉ (gọi khi chỉ cảm biến này đang bật XSHUT). */
bool vl53l0x_set_address(vl53l0x_t *d, uint8_t new_addr);
/* Khởi tạo đầy đủ: SPAD, tuning mặc định, hiệu chuẩn VHV/phase. */
bool vl53l0x_init(vl53l0x_t *d);
bool vl53l0x_set_timing_budget(vl53l0x_t *d, uint32_t budget_us);
/* Đo liên tục, không chờ giữa các lần đo. */
bool vl53l0x_start_continuous(vl53l0x_t *d);
/* Không chặn: trả về true khi có số đo mới. status: 11 = hợp lệ (theo datasheet RangeStatus). */
bool vl53l0x_poll(vl53l0x_t *d, uint16_t *mm, uint8_t *status);
uint8_t vl53l0x_model_id(vl53l0x_t *d);

#endif /* VL53L0X_H */
