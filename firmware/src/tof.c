/* USER CODE BEGIN Header */
/**
 * @file   tof.c
 * @brief  HAL I2C transport and XSHUT/address handling for up to 5 VL53L0X sensors.
 *         Full VL53L0X DataInit/StaticInit/calibration must be added before ranging.
 */
/* USER CODE END Header */

#include "tof.h"
#include "config.h"
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN 0 */
/* UART handle declared in main.c (CubeMX generated). */
extern UART_HandleTypeDef huart2;

/* ---------------------- VL53L0X register constants ---------------------- */
#define VL53L0X_DEFAULT_ADDR        (0x29U << 1)   /* 7-bit 0x29 → 8-bit 0x52 */
#define VL53L0X_REG_SYSRANGE_START  0x00U
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR 0x0BU
#define VL53L0X_REG_RESULT_RANGE_STATUS  0x14U
#define VL53L0X_REG_I2C_SLAVE_ADDR  0x8AU
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xC0U
#define VL53L0X_MODEL_ID           0xEEU
#define TOF_I2C_TIMEOUT_MS         20U

/* ---------------------- Private data ---------------------- */
static const uint8_t new_addrs[TOF_MAX_SENSORS] = {0x30U, 0x31U, 0x32U, 0x33U, 0x34U};
static uint8_t       dev_addrs_8bit[TOF_MAX_SENSORS]; /* HAL uses 8-bit addresses */
static uint8_t       dev_ok[TOF_MAX_SENSORS];

/* ---- XSHUT GPIO table ---- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} xshut_t;

static const xshut_t xshut[TOF_MAX_SENSORS] = {
    {XSHUT_LEFT_90_GPIO_Port,       XSHUT_LEFT_90_Pin},
    {XSHUT_DIAG_LEFT_45_GPIO_Port,  XSHUT_DIAG_LEFT_45_Pin},
    {XSHUT_FRONT_0_GPIO_Port,       XSHUT_FRONT_0_Pin},
    {XSHUT_DIAG_RIGHT_45_GPIO_Port, XSHUT_DIAG_RIGHT_45_Pin},
    {XSHUT_RIGHT_90_GPIO_Port,      XSHUT_RIGHT_90_Pin},
};

/* ---- Helpers ---- */
static void xshut_all_low(void)
{
    for (uint8_t i = 0; i < TOF_MAX_SENSORS; i++)
        HAL_GPIO_WritePin(xshut[i].port, xshut[i].pin, GPIO_PIN_RESET);
    HAL_Delay(10);
}

static void xshut_set(uint8_t idx, GPIO_PinState s)
{
    HAL_GPIO_WritePin(xshut[idx].port, xshut[idx].pin, s);
    HAL_Delay(s == GPIO_PIN_SET ? 10U : 2U);
}

static HAL_StatusTypeDef vl_write8(uint8_t addr8, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, addr8, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1U, TOF_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef vl_read(uint8_t addr8, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, addr8, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, TOF_I2C_TIMEOUT_MS);
}
/* USER CODE END 0 */

/* ================================================================== */

/**
 * @brief Bring sensors up one at a time and assign unique I2C addresses.
 * @return Bit mask of sensors found and assigned (bit 0 = sensor 0).
 *         This does not perform ST DataInit/StaticInit or calibration.
 */
uint8_t Tof_Init(void)
{
    /* USER CODE BEGIN Tof_Init */
    hi2c1.Init.ClockSpeed = 400000U;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) return 0U;

    /* 1. Pull all XSHUT low (all sensors in HW standby) */
    xshut_all_low();

    uint8_t online_mask = 0U;

    for (uint8_t i = 0; i < TOF_MAX_SENSORS; i++) {
        dev_ok[i] = 0U;
        dev_addrs_8bit[i] = 0U;
        /* 2. Bring this sensor out of standby */
        xshut_set(i, GPIO_PIN_SET);

        /* 3. Confirm that a VL53L0X, rather than any I2C responder, is present. */
        uint8_t model_id = 0U;
        if (HAL_I2C_IsDeviceReady(&hi2c1, VL53L0X_DEFAULT_ADDR, 2U,
                                  TOF_I2C_TIMEOUT_MS) != HAL_OK ||
            vl_read(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_IDENTIFICATION_MODEL_ID,
                    &model_id, 1U) != HAL_OK || model_id != VL53L0X_MODEL_ID) {
            xshut_set(i, GPIO_PIN_RESET);
            continue;
        }

        /* 4. Change I2C address */
        uint8_t new7 = new_addrs[i];
        if (vl_write8(VL53L0X_DEFAULT_ADDR, VL53L0X_REG_I2C_SLAVE_ADDR,
                      new7) != HAL_OK) {
            xshut_set(i, GPIO_PIN_RESET);
            continue;
        }

        /* Verify new address responds */
        uint8_t new_addr8 = (uint8_t)(new7 << 1);
        HAL_Delay(2U);
        if (HAL_I2C_IsDeviceReady(&hi2c1, new_addr8, 2U,
                                  TOF_I2C_TIMEOUT_MS) == HAL_OK &&
            vl_read(new_addr8, VL53L0X_REG_IDENTIFICATION_MODEL_ID,
                    &model_id, 1U) == HAL_OK && model_id == VL53L0X_MODEL_ID) {
            dev_addrs_8bit[i] = new_addr8;
            dev_ok[i] = 1U;
            online_mask |= (uint8_t)(1U << i);
        } else {
            /* Avoid leaving a failed sensor active at the shared default address. */
            xshut_set(i, GPIO_PIN_RESET);
        }
    }
    /* USER CODE END Tof_Init */
    return online_mask;
}

/* USER CODE BEGIN 1 */
uint8_t Tof_GetOnlineMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < TOF_MAX_SENSORS; i++) {
        if (dev_ok[i] != 0U) mask |= (uint8_t)(1U << i);
    }
    return mask;
}
/* USER CODE END 1 */

/**
 * @brief Perform a single ranging on each enabled sensor.
 * @param distances_mm  Output array [TOF_MAX_SENSORS]. -1 = error.
 */
void Tof_Range(int16_t *distances_mm)
{
    /* USER CODE BEGIN Tof_Range */
    if (distances_mm == NULL) return;

    for (uint8_t i = 0U; i < TOF_MAX_SENSORS; i++) distances_mm[i] = -1;

    for (uint8_t i = 0; i < TOF_MAX_SENSORS; i++) {
        if (!dev_ok[i]) continue;
        uint8_t addr = dev_addrs_8bit[i];

        /* Start from a cleared interrupt state, then trigger one measurement. */
        if (vl_write8(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U) != HAL_OK ||
            vl_write8(addr, VL53L0X_REG_SYSRANGE_START, 0x01U) != HAL_OK) {
            continue;
        }

        /* RESULT_RANGE_STATUS bit 0 is the data-ready flag. */
        uint8_t status = 0U;
        bool ready = false;
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) <= 100U) {
            if (vl_read(addr, VL53L0X_REG_RESULT_RANGE_STATUS, &status, 1U) != HAL_OK) {
                break;
            }
            if ((status & 0x01U) != 0U) {
                ready = true;
                break;
            }
            HAL_Delay(1U);
        }
        if (!ready) continue;

        /* Read result (12 bytes starting at 0x14, distance at offset 10-11) */
        uint8_t buf[12] = {0U};
        if (vl_read(addr, VL53L0X_REG_RESULT_RANGE_STATUS, buf, 12) == HAL_OK) {
            uint8_t range_status = (buf[0] & 0x78) >> 3; /* bits [6:3] */
            uint16_t mm = ((uint16_t)buf[10] << 8) | buf[11];
            /* Raw device status 0x0B maps to ST API's Range Valid (0). */
            if (range_status == 0x0BU && mm <= 2000U)
                distances_mm[i] = (int16_t)mm;
        }

        (void)vl_write8(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
    }
    /* USER CODE END Tof_Range */
}

/**
 * @brief Self-test: range all sensors and print results on UART2.
 */
void Tof_SelfTest(void)
{
    /* USER CODE BEGIN Tof_SelfTest */
    int16_t d[TOF_MAX_SENSORS];
    char buf[80];

    for (int rep = 0; rep < 5; rep++) {
        Tof_Range(d);
        for (uint8_t i = 0; i < TOF_MAX_SENSORS; i++) {
            if (d[i] >= 0)
                snprintf(buf, sizeof(buf), "TOF%d @0x%02X = %d mm\r\n",
                         (int)i, (unsigned)new_addrs[i], (int)d[i]);
            else
                snprintf(buf, sizeof(buf), "TOF%d @0x%02X ERR\r\n",
                         (int)i, (unsigned)new_addrs[i]);
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 100);
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)"---\r\n", 5, 50);
        HAL_Delay(500);
    }
    /* USER CODE END Tof_SelfTest */
}
