#include "mpu6050.h"

static bool wr(I2C_HandleTypeDef *i2c, uint8_t reg, uint8_t v)
{
  return HAL_I2C_Mem_Write(i2c, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 10) == HAL_OK;
}

uint8_t mpu6050_init(I2C_HandleTypeDef *i2c)
{
  uint8_t who = 0;
  if (HAL_I2C_Mem_Read(i2c, MPU6050_ADDR << 1, 0x75, I2C_MEMADD_SIZE_8BIT, &who, 1, 10) != HAL_OK) return 0;
  wr(i2c, 0x6B, 0x80); /* reset */
  HAL_Delay(100);
  wr(i2c, 0x6B, 0x01); /* thức dậy, clock theo gyro X */
  wr(i2c, 0x1A, 0x03); /* DLPF 42 Hz (gyro) / 44 Hz (accel) */
  wr(i2c, 0x19, 0x00); /* 1 kHz / (1 + 0) */
  wr(i2c, 0x1B, 0x08); /* ±500 dps */
  wr(i2c, 0x1C, 0x08); /* ±4 g */
  wr(i2c, 0x37, 0x10); /* INT: xoá khi đọc bất kỳ thanh ghi nào */
  wr(i2c, 0x38, 0x01); /* INT khi có dữ liệu mới */
  HAL_Delay(50);
  return who;
}

bool mpu6050_read(I2C_HandleTypeDef *i2c, mpu_raw_t *r)
{
  uint8_t b[14];
  if (HAL_I2C_Mem_Read(i2c, MPU6050_ADDR << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, b, 14, 5) != HAL_OK) return false;
  int16_t *o = &r->ax;
  for (int i = 0; i < 7; i++) o[i] = (int16_t)(b[2 * i] << 8 | b[2 * i + 1]);
  return true;
}
