/* Driver MPU6050: gyro ±500 dps, accel ±4 g, DLPF 42 Hz, lấy mẫu 1 kHz. */
#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define MPU6050_ADDR       0x68
#define MPU_GYRO_LSB_DPS   65.5f   /* ±500 dps */
#define MPU_ACC_LSB_G      8192.0f /* ±4 g */

typedef struct {
  int16_t ax, ay, az, temp, gx, gy, gz;
} mpu_raw_t;

/* Trả về WHO_AM_I (0x68 là hàng chuẩn; một số bản clone trả 0x70/0x72/0x98), 0 nếu không trả lời. */
uint8_t mpu6050_init(I2C_HandleTypeDef *i2c);
bool mpu6050_read(I2C_HandleTypeDef *i2c, mpu_raw_t *r);

#endif /* MPU6050_H */
