/* Firmware test linh kiện, điều khiển bằng trang web tools/hwtest_web (Web Serial qua USB-TTL hoặc HC-05).
 * Chỉ được biên dịch trong môi trường hwtest (-D HW_TEST). Code tạm thời: xoá khi test xong.
 *
 * Giao thức: mỗi dòng ASCII kết thúc bằng '\n'.
 *   Web -> xe:  ping | scan | tof on|off | mpu init|cal|zero|on|off | m <L%> <R%> | stop
 *               enc zero | go <0=L|1=R> <so_xung> <pct>
 *   Xe -> web:  HELLO | PONG | I2C,<bus>,<addr hex>... | TOFI,<i>,<addr>,<ok>
 *               TOF,<mm0..mm4>  (-1 = không hợp lệ) | MPUI,<who> | CALP,<%> | CALD,<bgx>,<bgy>,<bgz>,<std gz>
 *               IMU,<yaw>,<roll>,<pitch>,<gz> | ENC,<cntL>,<cntR>,<xung/s L>,<xung/s R> | GOD,<s>,<xung>
 *               LOG,<chu> */
#ifdef HW_TEST

#include "hwtest.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "mpu6050.h"
#include "vl53l0x.h"

extern I2C_HandleTypeDef hi2c1, hi2c2;
extern TIM_HandleTypeDef htim1, htim3, htim4;
extern UART_HandleTypeDef huart2;

#define PI_F          3.14159265f
#define TELEMETRY_MS  40  /* 25 Hz */
#define MOTOR_WDOG_MS 500 /* không nhận lệnh "m" trong 500 ms -> dừng (mất Bluetooth) */
enum { L = 0, R = 1 };

/* ========================= UART: nhận DMA vòng, gửi DMA ========================= */
static uint8_t rxbuf[256];
static uint32_t rx_tail;
static char txq[1024], txdma[1024];
static uint16_t txq_len;

static void tx_flush(void)
{
  if (!txq_len || huart2.gState != HAL_UART_STATE_READY) return;
  memcpy(txdma, txq, txq_len);
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)txdma, txq_len);
  txq_len = 0;
}

static void out(const char *fmt, ...)
{
  char b[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof b - 1) n = sizeof b - 1;
  if (txq_len + n > sizeof txq) return; /* đầy: bỏ dòng này */
  memcpy(txq + txq_len, b, n);
  txq_len += n;
  tx_flush();
}

static int rx_getc(void)
{
  uint32_t head = sizeof rxbuf - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
  if (head == rx_tail) return -1;
  uint8_t c = rxbuf[rx_tail];
  rx_tail = (rx_tail + 1) % sizeof rxbuf;
  return c;
}

static int read_line(char *line, int max)
{
  static char acc[64];
  static int n;
  int c;
  while ((c = rx_getc()) >= 0) {
    if (c == '\r' || c == '\n') {
      if (n == 0) continue;
      acc[n] = 0;
      strncpy(line, acc, max);
      n = 0;
      return 1;
    }
    if (n < (int)sizeof acc - 1) acc[n++] = (char)c;
  }
  return 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
  if (h != &huart2) return;
  HAL_UART_Receive_DMA(&huart2, rxbuf, sizeof rxbuf);
}

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Đếm chu kỳ CPU (tràn sau ~43 s ở 100 MHz, phép trừ không dấu vẫn đúng). */
static uint32_t cyc(void) { return DWT->CYCCNT; }
#define CYC_PER_MS (SystemCoreClock / 1000U)

/* ========================= encoder ========================= */
static int32_t enc_acc[2];
static uint16_t enc_last[2];

static void enc_update(void)
{
  uint16_t now[2] = {(uint16_t)htim3.Instance->CNT, (uint16_t)htim4.Instance->CNT};
  for (int i = 0; i < 2; i++) {
    int16_t d = (int16_t)(now[i] - enc_last[i]);
    enc_acc[i] += i == R ? -d : d; /* động cơ phải lắp đối xứng */
    enc_last[i] = now[i];
  }
}

/* ========================= động cơ (TB6612: A = phải, B = trái) ========================= */
static uint32_t motor_cmd_tick;

static void motor_pins(int s, GPIO_PinState in1, GPIO_PinState in2, uint32_t ccr)
{
  if (s == R) {
    HAL_GPIO_WritePin(AIN1_TB6612_GPIO_Port, AIN1_TB6612_Pin, in1);
    HAL_GPIO_WritePin(AIN2_TB6612_GPIO_Port, AIN2_TB6612_Pin, in2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
  } else {
    HAL_GPIO_WritePin(BIN1_TB6612_GPIO_Port, BIN1_TB6612_Pin, in1);
    HAL_GPIO_WritePin(BIN2_TB6612_GPIO_Port, BIN2_TB6612_Pin, in2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
  }
}

static void motor_set(int s, float pct)
{
  if (pct > 100) pct = 100;
  if (pct < -100) pct = -100;
  if (s == R) pct = -pct; /* động cơ phải lắp đối xứng: cùng dấu = cùng chạy tiến */
  uint32_t ccr = (uint32_t)(fabsf(pct) * (htim1.Init.Period + 1) / 100.0f);
  motor_pins(s, pct > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET, pct < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET, ccr);
}

static void motor_brake(int s) { motor_pins(s, GPIO_PIN_SET, GPIO_PIN_SET, htim1.Init.Period + 1); }

/* Chạy đúng N xung rồi phanh: để kiểm tra số xung / vòng bánh. */
static struct { int active, side; int32_t start, target; uint32_t done_tick; } go;

static void motor_stop_all(void)
{
  motor_set(L, 0);
  motor_set(R, 0);
  motor_cmd_tick = 0;
  go.active = 0;
}

/* ========================= ToF ========================= */
static GPIO_TypeDef *const TOF_PORT[5] = {TOF_L90_XSHUT_GPIO_Port, TOF_L45_XSHUT_GPIO_Port, TOF_M0_XSHUT_GPIO_Port,
                                          TOF_R45_XSHUT_GPIO_Port, TOF_R90_XSHUT_GPIO_Port};
static const uint16_t TOF_PIN[5] = {TOF_L90_XSHUT_Pin, TOF_L45_XSHUT_Pin, TOF_M0_XSHUT_Pin, TOF_R45_XSHUT_Pin,
                                    TOF_R90_XSHUT_Pin};
static vl53l0x_t tof[5];
static int tof_ready, tof_stream, tof_next;
static int16_t tof_mm[5] = {-1, -1, -1, -1, -1};

static void tof_init_all(void)
{
  for (int i = 0; i < 5; i++) HAL_GPIO_WritePin(TOF_PORT[i], TOF_PIN[i], GPIO_PIN_RESET);
  HAL_Delay(10);
  tof_ready = 0;
  for (int i = 0; i < 5; i++) {
    HAL_GPIO_WritePin(TOF_PORT[i], TOF_PIN[i], GPIO_PIN_SET);
    HAL_Delay(3);
    tof[i] = (vl53l0x_t){.i2c = &hi2c1, .addr = VL53L0X_DEFAULT_ADDR};
    int ok = 0;
    if (vl53l0x_set_address(&tof[i], 0x30 + i))
      ok = vl53l0x_init(&tof[i]) && vl53l0x_start_continuous(&tof[i]);
    else
      HAL_GPIO_WritePin(TOF_PORT[i], TOF_PIN[i], GPIO_PIN_RESET); /* tắt lại để không tranh địa chỉ 0x29 */
    if (ok) tof_ready |= 1 << i;
    out("TOFI,%d,%02X,%d\n", i, ok ? tof[i].addr : 0, ok);
    tx_flush();
    HAL_Delay(15); /* chờ DMA gửi xong dòng */
  }
}

/* Mỗi vòng lặp chỉ hỏi 1 cảm biến để không chặn MPU. */
static void tof_poll_one(void)
{
  int i = tof_next;
  tof_next = (tof_next + 1) % 5;
  if (!(tof_ready >> i & 1)) return;
  uint16_t mm;
  uint8_t st;
  if (vl53l0x_poll(&tof[i], &mm, &st)) tof_mm[i] = mm < 8000 ? (int16_t)mm : -1; /* 8190/8191 = ngoài tầm */
  (void)st;
}

/* ========================= MPU6050 ========================= */
static int mpu_ok, mpu_stream;
static uint32_t mpu_fail_run, mpu_recovers, mpu_last_ok;

/* Gỡ treo bus I2C2: MPU giữ SDA thấp sau một lần đọc hỏng (nhiễu động cơ).
 * Tạo 9 xung SCL + điều kiện STOP bằng GPIO rồi khởi tạo lại ngoại vi. */
static void i2c2_recover(void)
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
  mpu_recovers++;
}

/* Đọc MPU; hỏng 5 lần liên tiếp -> gỡ bus và cấu hình lại MPU. */
static int mpu_read_safe(mpu_raw_t *r)
{
  if (mpu6050_read(&hi2c2, r)) { mpu_fail_run = 0; mpu_last_ok = HAL_GetTick(); return 1; }
  if (++mpu_fail_run >= 5) {
    mpu_fail_run = 0;
    i2c2_recover();
    mpu6050_init(&hi2c2);
  }
  return 0;
}
static float gbias[3];
static struct { float yaw, roll, pitch, gz_f; } att;
static uint32_t mpu_t_prev; /* chu kỳ CPU */

static void mpu_cal(void)
{
  if (!mpu_ok) { out("LOG,Chua khoi tao MPU\n"); return; }
  motor_stop_all();
  double s[3] = {0}, q2 = 0;
  int n = 0;
  uint32_t t0 = HAL_GetTick(), next = cyc(), last_p = 0;
  const uint32_t dur = 3000;
  while (HAL_GetTick() - t0 < dur) {
    while ((int32_t)(cyc() - next) < 0) {}
    next += CYC_PER_MS;
    mpu_raw_t r;
    if (!mpu_read_safe(&r)) continue;
    float g[3] = {r.gx / MPU_GYRO_LSB_DPS, r.gy / MPU_GYRO_LSB_DPS, r.gz / MPU_GYRO_LSB_DPS};
    for (int i = 0; i < 3; i++) s[i] += g[i];
    q2 += (double)g[2] * g[2];
    n++;
    if (HAL_GetTick() - last_p >= 200) {
      last_p = HAL_GetTick();
      out("CALP,%lu\n", (unsigned long)((HAL_GetTick() - t0) * 100 / dur));
    }
  }
  if (!n) { out("LOG,MPU khong tra du lieu\n"); return; }
  for (int i = 0; i < 3; i++) gbias[i] = (float)(s[i] / n);
  float sd = sqrtf(fmaxf(0, (float)(q2 / n) - gbias[2] * gbias[2]));
  memset(&att, 0, sizeof att);
  mpu_t_prev = cyc();
  out("CALD,%.3f,%.3f,%.3f,%.4f\n", gbias[0], gbias[1], gbias[2], sd);
}

/* Bộ lọc bù: yaw tích phân gyro z, roll/pitch trộn gyro + accel. */
static void mpu_step(void)
{
  mpu_raw_t r;
  if (!mpu_read_safe(&r)) return;
  uint32_t t = cyc();
  float dt = (float)(t - mpu_t_prev) / SystemCoreClock;
  mpu_t_prev = t;
  if (dt <= 0 || dt > 0.05f) return;
  const float k = 0.98f;
  float gx = r.gx / MPU_GYRO_LSB_DPS - gbias[0];
  float gy = r.gy / MPU_GYRO_LSB_DPS - gbias[1];
  float gz = r.gz / MPU_GYRO_LSB_DPS - gbias[2];
  float ax = r.ax / MPU_ACC_LSB_G, ay = r.ay / MPU_ACC_LSB_G, az = r.az / MPU_ACC_LSB_G;
  att.gz_f += dt / (0.02f + dt) * (gz - att.gz_f);
  att.yaw += gz * dt;
  att.roll = k * (att.roll + gx * dt) + (1 - k) * atan2f(ay, az) * 180 / PI_F;
  att.pitch = k * (att.pitch + gy * dt) + (1 - k) * atan2f(-ax, sqrtf(ay * ay + az * az)) * 180 / PI_F;
}

/* ========================= lệnh ========================= */
static void cmd_scan(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c2, MPU6050_ADDR << 1, 2, 5) != HAL_OK) i2c2_recover();
  I2C_HandleTypeDef *bus[2] = {&hi2c1, &hi2c2};
  for (int b = 0; b < 2; b++) {
    char s[120];
    int n = snprintf(s, sizeof s, "I2C,%d", b + 1);
    for (uint8_t a = 1; a < 0x78 && n < (int)sizeof s - 4; a++)
      if (HAL_I2C_IsDeviceReady(bus[b], a << 1, 1, 2) == HAL_OK) n += snprintf(s + n, sizeof s - n, ",%02X", a);
    out("%s\n", s);
  }
}

static void dispatch(char *line)
{
  char *cmd = strtok(line, " ");
  char *a1 = strtok(NULL, " "), *a2 = strtok(NULL, " "), *a3 = strtok(NULL, " ");
  if (!cmd) return;
  if (!strcmp(cmd, "ping")) out("PONG\n");
  else if (!strcmp(cmd, "scan")) cmd_scan();
  else if (!strcmp(cmd, "tof") && a1) {
    if (!strcmp(a1, "on")) {
      if (!tof_ready) tof_init_all();
      tof_stream = 1;
    } else if (!strcmp(a1, "init")) {
      tof_init_all();
    } else {
      tof_stream = 0;
    }
  } else if (!strcmp(cmd, "mpu") && a1) {
    if (!strcmp(a1, "init")) {
      uint8_t who = mpu6050_init(&hi2c2);
      if (!who) { i2c2_recover(); who = mpu6050_init(&hi2c2); }
      mpu_ok = who != 0;
      mpu_t_prev = cyc();
      out("MPUI,%02X\n", who);
    } else if (!strcmp(a1, "cal")) mpu_cal();
    else if (!strcmp(a1, "zero")) att.yaw = 0;
    else if (!strcmp(a1, "on")) mpu_stream = mpu_ok;
    else mpu_stream = 0;
  } else if (!strcmp(cmd, "m") && a1 && a2) {
    go.active = 0;
    motor_set(L, strtof(a1, NULL));
    motor_set(R, strtof(a2, NULL));
    motor_cmd_tick = HAL_GetTick();
  } else if (!strcmp(cmd, "stop")) motor_stop_all();
  else if (!strcmp(cmd, "enc") && a1 && !strcmp(a1, "zero")) enc_acc[L] = enc_acc[R] = 0;
  else if (!strcmp(cmd, "go") && a1 && a2 && a3) {
    motor_stop_all();
    go.side = atoi(a1) ? R : L;
    go.start = enc_acc[go.side];
    go.target = labs(atol(a2));
    go.done_tick = 0;
    go.active = go.target > 0;
    if (go.active) motor_set(go.side, strtof(a3, NULL));
  } else out("LOG,Lenh la: %s\n", cmd);
}

/* Theo dõi lệnh "go": đủ xung thì phanh, chờ 400 ms cho bánh đứng hẳn rồi báo số xung thật. */
static void go_update(void)
{
  if (!go.active) return;
  int32_t d = labs(enc_acc[go.side] - go.start);
  if (!go.done_tick && d >= go.target) {
    motor_brake(go.side);
    go.done_tick = HAL_GetTick();
  }
  if (go.done_tick && HAL_GetTick() - go.done_tick > 400) {
    motor_set(go.side, 0);
    out("GOD,%d,%ld\n", go.side, (long)(enc_acc[go.side] - go.start));
    go.active = 0;
  }
}

void hwtest_run(void)
{
  dwt_init();
  HAL_UART_Receive_DMA(&huart2, rxbuf, sizeof rxbuf);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  motor_stop_all();
  HAL_GPIO_WritePin(STBY_TB6612_GPIO_Port, STBY_TB6612_Pin, GPIO_PIN_SET);
  enc_update();
  enc_acc[L] = enc_acc[R] = 0;
  out("HELLO\n");

  char line[64];
  uint32_t last_tel = 0, blink = 0, next_mpu = cyc();
  int32_t enc_prev[2] = {0, 0};
  while (1) {
    enc_update();
    go_update();
    if (motor_cmd_tick && HAL_GetTick() - motor_cmd_tick > MOTOR_WDOG_MS) motor_stop_all();
    if (mpu_ok && (int32_t)(cyc() - next_mpu) >= 0) {
      next_mpu = cyc() + CYC_PER_MS;
      mpu_step();
    }
    if (tof_ready) tof_poll_one();
    if (read_line(line, sizeof line)) dispatch(line);

    uint32_t now = HAL_GetTick();
    if (now - last_tel >= TELEMETRY_MS) {
      float k = 1000.0f / (now - last_tel);
      last_tel = now;
      out("ENC,%ld,%ld,%.0f,%.0f\n", (long)enc_acc[L], (long)enc_acc[R], (enc_acc[L] - enc_prev[L]) * k,
          (enc_acc[R] - enc_prev[R]) * k);
      enc_prev[L] = enc_acc[L];
      enc_prev[R] = enc_acc[R];
      if (tof_stream)
        out("TOF,%d,%d,%d,%d,%d\n", tof_mm[0], tof_mm[1], tof_mm[2], tof_mm[3], tof_mm[4]);
      if (mpu_stream)
        out("IMU,%.2f,%.2f,%.2f,%.2f,%d,%lu\n", att.yaw, att.roll, att.pitch, att.gz_f,
            HAL_GetTick() - mpu_last_ok < 200, (unsigned long)mpu_recovers);
    }
    tx_flush();
    if (now - blink >= 500) {
      blink = now;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
}

#endif /* HW_TEST */
