#include "tb6612.h"

#include <math.h>

#include "main.h"
#include "robot_config.h"

extern TIM_HandleTypeDef htim1;

static void pins(int side, GPIO_PinState in1, GPIO_PinState in2, uint32_t ccr)
{
  if (side == SIDE_R) {
    HAL_GPIO_WritePin(AIN1_TB6612_GPIO_Port, AIN1_TB6612_Pin, in1);
    HAL_GPIO_WritePin(AIN2_TB6612_GPIO_Port, AIN2_TB6612_Pin, in2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
  } else {
    HAL_GPIO_WritePin(BIN1_TB6612_GPIO_Port, BIN1_TB6612_Pin, in1);
    HAL_GPIO_WritePin(BIN2_TB6612_GPIO_Port, BIN2_TB6612_Pin, in2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
  }
}

void tb6612_init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  tb6612_stop_all();
  HAL_GPIO_WritePin(STBY_TB6612_GPIO_Port, STBY_TB6612_Pin, GPIO_PIN_SET);
}

void tb6612_set(int side, float pct)
{
  if (pct > 100) pct = 100;
  if (pct < -100) pct = -100;
  if (side == SIDE_R) pct = -pct; /* lắp đối xứng: cùng dấu = cùng chạy tiến */
  uint32_t ccr = (uint32_t)(fabsf(pct) * (htim1.Init.Period + 1) / 100.0f);
  pins(side, pct > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET, pct < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET, ccr);
}

void tb6612_brake(int side) { pins(side, GPIO_PIN_SET, GPIO_PIN_SET, htim1.Init.Period + 1); }

void tb6612_stop_all(void)
{
  tb6612_set(SIDE_L, 0);
  tb6612_set(SIDE_R, 0);
}
