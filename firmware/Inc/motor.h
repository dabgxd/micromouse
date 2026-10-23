/* USER CODE BEGIN Header */
/**
 * @file   motor.h
 * @brief  Interface for TB6612FNG motor driver.
 */
/* USER CODE END Header */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdbool.h>

/* Initialise driver (GPIO + PWM) */
void Motor_Init(void);

/* Set motor speed, percent 0..100. Negative values reverse direction. */
void Motor_SetSpeed(float left_percent, float right_percent);

/* Stop both motors and optionally standby */
void Motor_Stop(void);

/* Simple self‑test – spins motors for a short burst. */
bool Motor_SelfTest(void);

#endif // MOTOR_H_

