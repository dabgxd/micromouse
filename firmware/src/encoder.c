/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
/* USER CODE END Includes */

/* USER CODE BEGIN 0 */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define ENCODER_INPUT_FILTER 6U

static uint16_t previous_left = 0U;
static uint16_t previous_right = 0U;
static bool encoder_started = false;

static int32_t signed_delta16(uint16_t current, uint16_t previous)
{
    uint16_t difference = (uint16_t)(current - previous);
    if (difference <= INT16_MAX) return (int32_t)difference;
    return (int32_t)difference - 65536L;
}

bool Encoder_Init(void)
{
    TIM_Encoder_InitTypeDef config = {0};

    if (encoder_started) {
        (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
        (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
        encoder_started = false;
    }

    config.EncoderMode = TIM_ENCODERMODE_TI12;
    config.IC1Polarity = TIM_ICPOLARITY_RISING;
    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC1Prescaler = TIM_ICPSC_DIV1;
    config.IC1Filter = ENCODER_INPUT_FILTER;
    config.IC2Polarity = TIM_ICPOLARITY_RISING;
    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC2Prescaler = TIM_ICPSC_DIV1;
    config.IC2Filter = ENCODER_INPUT_FILTER;

    if (HAL_TIM_Encoder_Init(&htim3, &config) != HAL_OK ||
        HAL_TIM_Encoder_Init(&htim4, &config) != HAL_OK) {
        return false;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);

    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
        return false;
    }
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
        (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
        return false;
    }

    previous_left = 0U;
    previous_right = 0U;
    encoder_started = true;
    return true;
}

void Encoder_GetDelta(int32_t *left_delta, int32_t *right_delta)
{
    if (left_delta != NULL) *left_delta = 0;
    if (right_delta != NULL) *right_delta = 0;
    if (left_delta == NULL || right_delta == NULL) return;
    if (!encoder_started) return;

    uint16_t current_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint16_t current_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

    *left_delta = signed_delta16(current_left, previous_left);
    *right_delta = signed_delta16(current_right, previous_right);

    previous_left = current_left;
    previous_right = current_right;
}

void Encoder_Reset(void)
{
    if (!encoder_started) return;

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    previous_left = 0U;
    previous_right = 0U;
}
/* USER CODE END 0 */
