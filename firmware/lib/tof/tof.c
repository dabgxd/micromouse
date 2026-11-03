#include "tof.h"

#include "main.h"
#include "vl53l0x.h"

extern I2C_HandleTypeDef hi2c1;

const char *const TOF_NAME[TOF_COUNT] = {"L90", "L45", "M0", "R45", "R90"};

static GPIO_TypeDef *const XSHUT_PORT[TOF_COUNT] = {TOF_L90_XSHUT_GPIO_Port, TOF_L45_XSHUT_GPIO_Port,
                                                    TOF_M0_XSHUT_GPIO_Port, TOF_R45_XSHUT_GPIO_Port,
                                                    TOF_R90_XSHUT_GPIO_Port};
static const uint16_t XSHUT_PIN[TOF_COUNT] = {TOF_L90_XSHUT_Pin, TOF_L45_XSHUT_Pin, TOF_M0_XSHUT_Pin,
                                              TOF_R45_XSHUT_Pin, TOF_R90_XSHUT_Pin};

static vl53l0x_t dev[TOF_COUNT];
static int ready, next;
static int16_t mm_last[TOF_COUNT] = {-1, -1, -1, -1, -1};

int tof_init_all(tof_init_cb cb)
{
  for (int i = 0; i < TOF_COUNT; i++) HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_RESET);
  HAL_Delay(10);
  ready = 0;
  for (int i = 0; i < TOF_COUNT; i++) {
    mm_last[i] = -1;
    HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_SET);
    HAL_Delay(3);
    dev[i] = (vl53l0x_t){.i2c = &hi2c1, .addr = VL53L0X_DEFAULT_ADDR};
    int ok = 0;
    if (vl53l0x_set_address(&dev[i], TOF_ADDR_BASE + i))
      ok = vl53l0x_init(&dev[i]) && vl53l0x_start_continuous(&dev[i]);
    else
      HAL_GPIO_WritePin(XSHUT_PORT[i], XSHUT_PIN[i], GPIO_PIN_RESET); /* tắt lại để không tranh địa chỉ 0x29 */
    if (ok) ready |= 1 << i;
    if (cb) cb(i, ok ? dev[i].addr : 0, ok);
  }
  return ready;
}

void tof_poll_one(void)
{
  int i = next;
  next = (next + 1) % TOF_COUNT;
  if (!(ready >> i & 1)) return;
  uint16_t mm;
  uint8_t status;
  if (vl53l0x_poll(&dev[i], &mm, &status)) mm_last[i] = mm < 8000 ? (int16_t)mm : -1; /* 8190/8191 = ngoài tầm */
}

int16_t tof_mm(int idx) { return mm_last[idx]; }

int tof_ready_mask(void) { return ready; }
