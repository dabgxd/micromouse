#include "encoder.h"

#include "main.h"
#include "robot_config.h"

extern TIM_HandleTypeDef htim3, htim4;

static int32_t acc[2];
static uint16_t last[2];

void encoder_init(void)
{
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  encoder_update();
  encoder_zero();
}

void encoder_update(void)
{
  uint16_t now[2] = {(uint16_t)htim3.Instance->CNT, (uint16_t)htim4.Instance->CNT};
  for (int i = 0; i < 2; i++) {
    int16_t d = (int16_t)(now[i] - last[i]);
    acc[i] += i == SIDE_R ? -d : d;
    last[i] = now[i];
  }
}

int32_t encoder_count(int side) { return acc[side]; }

void encoder_zero(void) { acc[SIDE_L] = acc[SIDE_R] = 0; }
