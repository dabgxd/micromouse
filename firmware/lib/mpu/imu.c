#include "imu.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "mpu6050.h"
#include "timebase.h"

#define PI_F 3.14159265f

extern I2C_HandleTypeDef hi2c2;

static int ok;
static uint32_t fail_run, recovers, last_ok_tick;
static float bias[3], scale = 1.0f;
static imu_att_t att;
static float gz_prev;
static int still;
static uint32_t still_since, t_prev; /* t_prev: chu kỳ CPU */

/* Tạo 9 xung SCL + điều kiện STOP bằng GPIO rồi khởi tạo lại ngoại vi I2C2. */
void imu_bus_recover(void)
{
  HAL_I2C_DeInit(&hi2c2);
  GPIO_InitTypeDef g = {.Mode = GPIO_MODE_OUTPUT_OD, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_FREQ_HIGH};
  g.Pin = GPIO_PIN_10; HAL_GPIO_Init(GPIOB, &g); /* SCL */
  g.Pin = GPIO_PIN_3;  HAL_GPIO_Init(GPIOB, &g); /* SDA */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_10, GPIO_PIN_SET);
  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 200; d++) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    for (volatile int d = 0; d < 200; d++) {}
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); /* STOP: SDA lên khi SCL cao */
  for (volatile int d = 0; d < 200; d++) {}
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  hi2c2.Instance->CR1 |= I2C_CR1_SWRST;
  hi2c2.Instance->CR1 &= ~I2C_CR1_SWRST;
  HAL_I2C_Init(&hi2c2); /* MspInit đặt lại chân về AF */
  recovers++;
}

/* Đọc MPU; hỏng 5 lần liên tiếp -> gỡ bus và cấu hình lại MPU. */
static int read_safe(mpu_raw_t *r)
{
  if (mpu6050_read(&hi2c2, r)) {
    fail_run = 0;
    last_ok_tick = HAL_GetTick();
    return 1;
  }
  if (++fail_run >= 5) {
    fail_run = 0;
    imu_bus_recover();
    mpu6050_init(&hi2c2);
  }
  return 0;
}

uint8_t imu_init(void)
{
  uint8_t who = mpu6050_init(&hi2c2);
  if (!who) {
    imu_bus_recover();
    who = mpu6050_init(&hi2c2);
  }
  ok = who != 0;
  t_prev = cyc_now();
  return who;
}

int imu_ok(void) { return ok; }

int imu_calibrate(uint32_t ms, imu_progress_cb cb, float *noise_out)
{
  if (!ok) return 0;
  double s[3] = {0}, q2 = 0;
  int n = 0;
  uint32_t t0 = HAL_GetTick(), next = cyc_now(), last_p = 0;
  while (HAL_GetTick() - t0 < ms) {
    while ((int32_t)(cyc_now() - next) < 0) {}
    next += CYC_PER_MS;
    mpu_raw_t r;
    if (!read_safe(&r)) continue;
    float g[3] = {r.gx / MPU_GYRO_LSB_DPS, r.gy / MPU_GYRO_LSB_DPS, r.gz / MPU_GYRO_LSB_DPS};
    for (int i = 0; i < 3; i++) s[i] += g[i];
    q2 += (double)g[2] * g[2];
    n++;
    if (cb && HAL_GetTick() - last_p >= 200) {
      last_p = HAL_GetTick();
      cb((int)((HAL_GetTick() - t0) * 100 / ms));
    }
  }
  if (!n) return 0;
  for (int i = 0; i < 3; i++) bias[i] = (float)(s[i] / n);
  if (noise_out) *noise_out = sqrtf(fmaxf(0, (float)(q2 / n) - bias[2] * bias[2]));
  memset(&att, 0, sizeof att);
  gz_prev = 0;
  t_prev = cyc_now();
  return 1;
}

void imu_step(int moving)
{
  if (!ok) return;
  mpu_raw_t r;
  if (!read_safe(&r)) return;
  uint32_t t = cyc_now();
  float dt = (float)(t - t_prev) / SystemCoreClock;
  t_prev = t;
  if (dt <= 0 || dt > 0.05f) return;
  const float k = 0.98f;
  float gx = r.gx / MPU_GYRO_LSB_DPS - bias[0];
  float gy = r.gy / MPU_GYRO_LSB_DPS - bias[1];
  float gz_raw = r.gz / MPU_GYRO_LSB_DPS;
  float gz = (gz_raw - bias[2]) * scale;
  float ax = r.ax / MPU_ACC_LSB_G, ay = r.ay / MPU_ACC_LSB_G, az = r.az / MPU_ACC_LSB_G;
  att.gz += dt / (0.02f + dt) * (gz - att.gz);

  uint32_t now = HAL_GetTick();
  int quiet = !moving && fabsf(att.gz) < 1.5f;
  if (!quiet) still_since = now;
  still = quiet && now - still_since > 300;
  if (still)
    bias[2] += 0.002f * (gz_raw - bias[2]); /* hằng số thời gian ~0,5 s */
  else
    att.yaw += 0.5f * (gz + gz_prev) * dt; /* tích phân hình thang */
  gz_prev = still ? 0 : gz;
  att.roll = k * (att.roll + gx * dt) + (1 - k) * atan2f(ay, az) * 180 / PI_F;
  att.pitch = k * (att.pitch + gy * dt) + (1 - k) * atan2f(-ax, sqrtf(ay * ay + az * az)) * 180 / PI_F;
}

void imu_zero_yaw(void) { att.yaw = 0; }
void imu_set_scale(float k) { scale = k; }
const imu_att_t *imu_att(void) { return &att; }
void imu_get_bias(float b[3]) { memcpy(b, bias, sizeof bias); }
int imu_still(void) { return still; }
int imu_live(void) { return HAL_GetTick() - last_ok_tick < 200; }
uint32_t imu_recover_count(void) { return recovers; }
