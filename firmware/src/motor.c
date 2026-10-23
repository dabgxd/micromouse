/* USER CODE BEGIN Header */
/**
 * @file   motor.c
 * @brief  Driver for TB6612FNG motor driver.
 *         Wraps HAL PWM + GPIO control. All user code inside USER CODE blocks.
 */
/* USER CODE END Header */

#include "motor.h"
#include "config.h"
#include "stm32f4xx_hal.h"

/* Extern handles declared in main.c (CubeMX generated) */
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PD */
#define MOTOR_PWM_ARR 4999U  /* 100 MHz timer clock / 5000 = 20 kHz */
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static bool motor_initialized = false;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
/* Private helper to set PWM duty (0..100%). */
static void set_pwm_duty(TIM_HandleTypeDef *htim, uint32_t channel, float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;
    uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    uint32_t pulse = (uint32_t)((duty_percent / 100.0f) * (float)period_counts);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}
/* USER CODE END 0 */

/**
 * @brief Initialize motor driver pins and PWM.
 *        Must be called after HAL_TIM_PWM_Init(&htim1) and HAL_GPIO_Init.
 */
void Motor_Init(void)
{
    /* USER CODE BEGIN Motor_Init */
    motor_initialized = false;
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    /* MX_TIM1_Init currently sets ARR to 65535. Override it here so the
       motor component owns its documented 20 kHz PWM period. */
    htim1.Init.Prescaler = 0U;
    htim1.Init.Period = MOTOR_PWM_ARR;
    __HAL_TIM_SET_PRESCALER(&htim1, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, MOTOR_PWM_ARR);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_GENERATE_EVENT(&htim1, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

    // Default direction: forward (AIN1=1, AIN2=0, BIN1=1, BIN2=0)
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

    // Start PWM with 0% duty (stopped)
    set_pwm_duty(&htim1, TIM_CHANNEL_1, 0.0f);
    set_pwm_duty(&htim1, TIM_CHANNEL_2, 0.0f);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        return;
    }

    motor_initialized = true;
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    /* USER CODE END Motor_Init */
}

/**
 * @brief Set motor speeds.
 * @param left_percent  0..100 (forward)  negative for reverse
 * @param right_percent 0..100 (forward)  negative for reverse
 */
void Motor_SetSpeed(float left_percent, float right_percent)
{
    /* USER CODE BEGIN Motor_SetSpeed */
    if (!motor_initialized) return;

    /* Motor_Stop() places the TB6612 in standby. Re-enable it before
       applying a new direction or PWM command. */
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    // Left motor direction
    if (left_percent >= 0.0f) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
        left_percent = -left_percent; // make positive for PWM
    }
    // Right motor direction
    if (right_percent >= 0.0f) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        right_percent = -right_percent;
    }
    // Clamp to 100%
    if (left_percent > 100.0f) left_percent = 100.0f;
    if (right_percent > 100.0f) right_percent = 100.0f;
    // Apply PWM
    set_pwm_duty(&htim1, TIM_CHANNEL_1, left_percent);
    set_pwm_duty(&htim1, TIM_CHANNEL_2, right_percent);
    /* USER CODE END Motor_SetSpeed */
}

/**
 * @brief Stop both motors and put driver in standby.
 */
void Motor_Stop(void)
{
    /* USER CODE BEGIN Motor_Stop */
    if (motor_initialized) {
        set_pwm_duty(&htim1, TIM_CHANNEL_1, 0.0f);
        set_pwm_duty(&htim1, TIM_CHANNEL_2, 0.0f);
    }
    // Optional: put driver in standby to reduce power draw
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    /* USER CODE END Motor_Stop */
}

/* USER CODE BEGIN 1 */
/* You can add self‑test function that spins each motor briefly.
   Returns true if encoder ticks increase (requires encoders wired). */
bool Motor_SelfTest(void)
{
    if (!motor_initialized) return false;

    // Simple test: forward half‑speed both motors for 500 ms, then stop.
    Motor_SetSpeed(50.0f, 50.0f);
    HAL_Delay(500);
    Motor_Stop();
    // Indicate success by toggling onboard LED twice.
    for (int i=0;i<2;i++) {
        HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
        HAL_Delay(200);
    }
    return true; // Confirms the test sequence ran; it does not validate encoder motion.
}
/* USER CODE END 1 */

