/* USER CODE BEGIN Header */
/**
 * @file   mpu6050.h
 * @brief  Minimal driver interface for MPU6050 IMU.
 */
/* USER CODE END Header */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

/* Initialise the sensor and enable EXTI9_5. I2C2 and PB5 GPIO must be ready. */
bool Mpu6050_Init(void);

/* Read raw Z-axis gyro value; returns 0 if the I2C read fails. */
int16_t Mpu6050_ReadGyroZ(void);

/* Read raw Z-axis gyro value and report I2C errors explicitly. */
bool Mpu6050_TryReadGyroZ(int16_t *out_raw);

/* ISR hook – call from EXTI5_IRQHandler(). */
void Mpu6050_DataReadyISR(void);

/* Poll for new data – returns true when a fresh sample was read. */
bool Mpu6050_Poll(int16_t *out_raw);

/* Simple self‑test that prints readings on UART2. */
void Mpu6050_SelfTest(void);

#endif // MPU6050_H_

