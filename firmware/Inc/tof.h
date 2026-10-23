/* USER CODE BEGIN Header */
/**
 * @file   tof.h
 * @brief  Interface for VL53L0X (up to 5 sensors).
 */
/* USER CODE END Header */

#ifndef TOF_H_
#define TOF_H_

#include <stdint.h>

#define TOF_MAX_SENSORS 5

/*
 * Power up sensors one at a time and assign addresses.
 * Returns a bit mask of sensors detected (bit 0 = sensor 0).
 * Requires I2C1 and XSHUT GPIOs to be configured. This does not run the
 * full ST VL53L0X initialization/calibration sequence.
 */
uint8_t Tof_Init(void);

/* Return the current sensor-presence mask. */
uint8_t Tof_GetOnlineMask(void);

/* Perform a single ranging on every sensor.
 * distances_mm – array of length TOF_MAX_SENSORS, filled with measured distance
 *   in millimetres, or -1 on error/out‑of‑range.
 */
void Tof_Range(int16_t *distances_mm);

/* Simple self‑test that prints results on UART2 (USART2). */
void Tof_SelfTest(void);

#endif // TOF_H_

