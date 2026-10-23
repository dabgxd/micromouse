/* USER CODE BEGIN Header */
/**
 * @file   mpu6050.c
 * @brief  Minimal driver for MPU6050 IMU (gyro Z axis). Uses I2C2.
 *         Handles init, data ready interrupt, and raw angular rate read.
 */
/* USER CODE END Header */

#include "mpu6050.h"
#include "config.h"
/* USER CODE BEGIN Includes */
#include "i2c.h"
/* USER CODE END Includes */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>

/* UART handle declared in main.c (CubeMX generated). */
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN PD */
/* MPU6050 register map (simplified) */
#define MPU6050_ADDR          (0x68U << 1)   // 7-bit address shifted for HAL
#define REG_PWR_MGMT_1        0x6B
#define REG_SMPLRT_DIV        0x19
#define REG_CONFIG            0x1A
#define REG_GYRO_CONFIG       0x1B
#define REG_INT_ENABLE        0x38
#define REG_GYRO_ZOUT_H       0x47
#define MPU_I2C_TIMEOUT_MS    20U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
/* Private flag set by EXTI5 ISR */
static volatile uint8_t mpu_data_ready = 0;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static HAL_StatusTypeDef mpu_write8(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, MPU_I2C_TIMEOUT_MS);
}
/* USER CODE END 0 */

/**
 * @brief Initialise MPU6050.
 *        Sets a 1 kHz sample rate, full-scale ±2000 °/s and data-ready interrupt.
 * @return true when the sensor setup succeeds.
 */
bool Mpu6050_Init(void)
{
    /* USER CODE BEGIN Mpu6050_Init */
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    hi2c2.Init.ClockSpeed = 400000U;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) return false;

    if (HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDR, 3U,
                              MPU_I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    /* Wake the device and select the X-gyro PLL as the clock source. */
    if (mpu_write8(REG_PWR_MGMT_1, 0x01U) != HAL_OK ||
        mpu_write8(REG_CONFIG, 0x03U) != HAL_OK ||
        /* DLPF enabled: gyro output rate is 1 kHz; divider 0 keeps 1 kHz. */
        mpu_write8(REG_SMPLRT_DIV, 0x00U) != HAL_OK ||
        mpu_write8(REG_GYRO_CONFIG, 0x18U) != HAL_OK ||
        mpu_write8(REG_INT_ENABLE, 0x01U) != HAL_OK) {
        return false;
    }

    mpu_data_ready = 0U;
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    /* USER CODE END Mpu6050_Init */
    return true;
}

/**
 * @brief Read angular rate around Z axis (raw 16‑bit value).
 * @return int16_t raw gyro Z (LSB = 16.4 LSB/°/s for ±2000 °/s)
 */
int16_t Mpu6050_ReadGyroZ(void)
{
    /* USER CODE BEGIN Mpu6050_ReadGyroZ */
    int16_t raw = 0;
    (void)Mpu6050_TryReadGyroZ(&raw);
    return raw;
    /* USER CODE END Mpu6050_ReadGyroZ */
}

/* USER CODE BEGIN 1 */
bool Mpu6050_TryReadGyroZ(int16_t *out_raw)
{
    if (out_raw == NULL) return false;

    uint8_t buf[2] = {0U, 0U};
    if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, REG_GYRO_ZOUT_H,
                         I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                         MPU_I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    *out_raw = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    return true;
}

/**
 * @brief Called from EXTI5 ISR to flag new data.
 */
void Mpu6050_DataReadyISR(void)
{
    mpu_data_ready = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin) {
        Mpu6050_DataReadyISR();
    }
}

/**
 * @brief Poll for new gyro data – clears flag and returns latest value.
 * @param[out] out_raw Raw Z value, valid only if function returns true.
 * @return true if new data was available.
 */
bool Mpu6050_Poll(int16_t *out_raw)
{
    if (out_raw == NULL) return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool ready = (mpu_data_ready != 0U);
    mpu_data_ready = 0U;
    if (primask == 0U) __enable_irq();

    return ready && Mpu6050_TryReadGyroZ(out_raw);
}
/* USER CODE END 1 */

/**
 * @brief Simple self‑test: prints gyro Z every 200 ms for 2 seconds on UART2.
 */
void Mpu6050_SelfTest(void)
{
    /* USER CODE BEGIN Mpu6050_SelfTest */
    char buf[64];
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 2000) {
        int16_t raw;
        if (!Mpu6050_TryReadGyroZ(&raw)) {
            snprintf(buf, sizeof(buf), "MPU I2C read error\r\n");
        } else {
            float dps = raw / 16.4f; // convert to °/s (FS=2000)
            snprintf(buf, sizeof(buf), "MPU Z = %d (%.2f dps)\r\n", raw, dps);
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
        HAL_Delay(200);
    }
    /* USER CODE END Mpu6050_SelfTest */
}

/* Additional helper functions can be added here. */

