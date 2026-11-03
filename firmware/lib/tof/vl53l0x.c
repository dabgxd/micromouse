/* Driver VL53L0X tối giản, theo trình tự khởi tạo của thư viện Pololu VL53L0X (MIT). */
#include "vl53l0x.h"

enum {
  SYSRANGE_START = 0x00,
  SYSTEM_SEQUENCE_CONFIG = 0x01,
  SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
  SYSTEM_INTERRUPT_CLEAR = 0x0B,
  RESULT_INTERRUPT_STATUS = 0x13,
  RESULT_RANGE_STATUS = 0x14,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
  MSRC_CONFIG_TIMEOUT_MACROP = 0x46,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
  PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
  MSRC_CONFIG_CONTROL = 0x60,
  FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
  GPIO_HV_MUX_ACTIVE_HIGH = 0x84,
  VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89,
  I2C_SLAVE_DEVICE_ADDRESS = 0x8A,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
  GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
  IDENTIFICATION_MODEL_ID = 0xC0,
};

#define IO_TIMEOUT_MS 50

/* ---- I2C ---- */
static void wr(vl53l0x_t *d, uint8_t reg, uint8_t v)
{
  if (HAL_I2C_Mem_Write(d->i2c, d->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 10) != HAL_OK) d->ok = false;
}

static void wr16(vl53l0x_t *d, uint8_t reg, uint16_t v)
{
  uint8_t b[2] = {v >> 8, v & 0xFF};
  if (HAL_I2C_Mem_Write(d->i2c, d->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, b, 2, 10) != HAL_OK) d->ok = false;
}

static void wrn(vl53l0x_t *d, uint8_t reg, uint8_t *b, uint16_t n)
{
  if (HAL_I2C_Mem_Write(d->i2c, d->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, b, n, 10) != HAL_OK) d->ok = false;
}

static uint8_t rd(vl53l0x_t *d, uint8_t reg)
{
  uint8_t v = 0;
  if (HAL_I2C_Mem_Read(d->i2c, d->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 10) != HAL_OK) d->ok = false;
  return v;
}

static uint16_t rd16(vl53l0x_t *d, uint8_t reg)
{
  uint8_t b[2] = {0, 0};
  if (HAL_I2C_Mem_Read(d->i2c, d->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, b, 2, 10) != HAL_OK) d->ok = false;
  return (uint16_t)(b[0] << 8 | b[1]);
}

static void rdn(vl53l0x_t *d, uint8_t reg, uint8_t *b, uint16_t n)
{
  if (HAL_I2C_Mem_Read(d->i2c, d->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, b, n, 10) != HAL_OK) d->ok = false;
}

/* ---- timing ---- */
static uint16_t decode_timeout(uint16_t v) { return (uint16_t)(((v & 0xFF) << (v >> 8)) + 1); }

static uint16_t encode_timeout(uint32_t mclks)
{
  if (mclks == 0) return 0;
  uint32_t ls = mclks - 1;
  uint16_t ms = 0;
  while (ls & 0xFFFFFF00) { ls >>= 1; ms++; }
  return (uint16_t)((ms << 8) | (ls & 0xFF));
}

static uint32_t macro_ns(uint8_t vcsel_pclks) { return ((2304UL * vcsel_pclks * 1655UL) + 500) / 1000; }

static uint32_t mclks_to_us(uint16_t mclks, uint8_t vcsel)
{
  uint32_t m = macro_ns(vcsel);
  return ((mclks * m) + 500) / 1000;
}

static uint32_t us_to_mclks(uint32_t us, uint8_t vcsel)
{
  uint32_t m = macro_ns(vcsel);
  return ((us * 1000) + (m / 2)) / m;
}

typedef struct { bool tcc, msrc, dss, pre_range, final_range; } seq_en_t;
typedef struct {
  uint8_t pre_vcsel, final_vcsel;
  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} seq_to_t;

static void get_seq(vl53l0x_t *d, seq_en_t *e, seq_to_t *t)
{
  uint8_t s = rd(d, SYSTEM_SEQUENCE_CONFIG);
  e->tcc = (s >> 4) & 1; e->dss = (s >> 3) & 1; e->msrc = (s >> 2) & 1;
  e->pre_range = (s >> 6) & 1; e->final_range = (s >> 7) & 1;

  t->pre_vcsel = (uint8_t)((rd(d, PRE_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1);
  t->msrc_dss_tcc_mclks = rd(d, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  t->msrc_dss_tcc_us = mclks_to_us(t->msrc_dss_tcc_mclks, t->pre_vcsel);
  t->pre_range_mclks = decode_timeout(rd16(d, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  t->pre_range_us = mclks_to_us(t->pre_range_mclks, t->pre_vcsel);
  t->final_vcsel = (uint8_t)((rd(d, FINAL_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1);
  t->final_range_mclks = decode_timeout(rd16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
  t->final_range_us = mclks_to_us(t->final_range_mclks, t->final_vcsel);
}

/* Chi phí cố định (µs) của từng bước đo, theo API của ST. */
enum { START_OH = 1910, END_OH = 960, MSRC_OH = 660, TCC_OH = 590, DSS_OH = 690, PRE_OH = 660, FINAL_OH = 550 };

static uint32_t overhead_us(const seq_en_t *e, const seq_to_t *t)
{
  uint32_t used = START_OH + END_OH;
  if (e->tcc) used += t->msrc_dss_tcc_us + TCC_OH;
  if (e->dss) used += 2 * (t->msrc_dss_tcc_us + DSS_OH);
  else if (e->msrc) used += t->msrc_dss_tcc_us + MSRC_OH;
  if (e->pre_range) used += t->pre_range_us + PRE_OH;
  return used;
}

static uint32_t get_budget(vl53l0x_t *d)
{
  seq_en_t e;
  seq_to_t t;
  get_seq(d, &e, &t);
  uint32_t b = overhead_us(&e, &t);
  if (e.final_range) b += t.final_range_us + FINAL_OH;
  return b;
}

bool vl53l0x_set_timing_budget(vl53l0x_t *d, uint32_t budget_us)
{
  seq_en_t e;
  seq_to_t t;
  get_seq(d, &e, &t);
  if (!e.final_range) return false;
  uint32_t used = overhead_us(&e, &t) + FINAL_OH;
  if (used > budget_us) return false;
  uint32_t mclks = us_to_mclks(budget_us - used, t.final_vcsel);
  if (e.pre_range) mclks += t.pre_range_mclks;
  wr16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(mclks));
  d->budget_us = budget_us;
  return d->ok;
}

/* ---- khởi tạo ---- */
static bool wait_reg(vl53l0x_t *d, uint8_t reg, uint8_t mask, bool until_nonzero)
{
  uint32_t t0 = HAL_GetTick();
  while (((rd(d, reg) & mask) != 0) != until_nonzero) {
    if (!d->ok || HAL_GetTick() - t0 > IO_TIMEOUT_MS) return false;
  }
  return true;
}

static bool get_spad_info(vl53l0x_t *d, uint8_t *count, bool *aperture)
{
  wr(d, 0x80, 0x01); wr(d, 0xFF, 0x01); wr(d, 0x00, 0x00);
  wr(d, 0xFF, 0x06); wr(d, 0x83, rd(d, 0x83) | 0x04);
  wr(d, 0xFF, 0x07); wr(d, 0x81, 0x01);
  wr(d, 0x80, 0x01);
  wr(d, 0x94, 0x6B); wr(d, 0x83, 0x00);
  if (!wait_reg(d, 0x83, 0xFF, true)) return false;
  wr(d, 0x83, 0x01);
  uint8_t tmp = rd(d, 0x92);
  *count = tmp & 0x7F;
  *aperture = (tmp >> 7) & 1;
  wr(d, 0x81, 0x00); wr(d, 0xFF, 0x06);
  wr(d, 0x83, rd(d, 0x83) & ~0x04);
  wr(d, 0xFF, 0x01); wr(d, 0x00, 0x01);
  wr(d, 0xFF, 0x00); wr(d, 0x80, 0x00);
  return d->ok;
}

static bool single_ref_cal(vl53l0x_t *d, uint8_t vhv_init)
{
  wr(d, SYSRANGE_START, 0x01 | vhv_init);
  if (!wait_reg(d, RESULT_INTERRUPT_STATUS, 0x07, true)) return false;
  wr(d, SYSTEM_INTERRUPT_CLEAR, 0x01);
  wr(d, SYSRANGE_START, 0x00);
  return d->ok;
}

static const uint8_t TUNING[][2] = {
  {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01},
  {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00}, {0x30, 0x20}, {0xFF, 0x00},
  {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00},
  {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00}, {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
  {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14},
  {0x49, 0xFF}, {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21}, {0xFF, 0x01},
  {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40}, {0x0E, 0x06},
  {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
  {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00}, {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08},
  {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00},
  {0xFF, 0x01}, {0x0D, 0x01}, {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01},
  {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
};

uint8_t vl53l0x_model_id(vl53l0x_t *d) { return rd(d, IDENTIFICATION_MODEL_ID); }

bool vl53l0x_set_address(vl53l0x_t *d, uint8_t new_addr)
{
  d->ok = true;
  wr(d, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
  if (d->ok) d->addr = new_addr;
  return d->ok;
}

bool vl53l0x_init(vl53l0x_t *d)
{
  d->ok = true;
  if (rd(d, IDENTIFICATION_MODEL_ID) != 0xEE) return d->ok = false;

  /* cảm biến chạy 2V8 */
  wr(d, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, rd(d, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01);
  wr(d, 0x88, 0x00);
  wr(d, 0x80, 0x01); wr(d, 0xFF, 0x01); wr(d, 0x00, 0x00);
  d->stop_variable = rd(d, 0x91);
  wr(d, 0x00, 0x01); wr(d, 0xFF, 0x00); wr(d, 0x80, 0x00);

  wr(d, MSRC_CONFIG_CONTROL, rd(d, MSRC_CONFIG_CONTROL) | 0x12);
  wr16(d, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * 128)); /* 0.25 MCPS */
  wr(d, SYSTEM_SEQUENCE_CONFIG, 0xFF);

  /* SPAD tham chiếu */
  uint8_t spad_count;
  bool aperture;
  if (!get_spad_info(d, &spad_count, &aperture)) return false;
  uint8_t map[6];
  rdn(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);
  wr(d, 0xFF, 0x01);
  wr(d, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  wr(d, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  wr(d, 0xFF, 0x00);
  wr(d, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
  uint8_t first = aperture ? 12 : 0, enabled = 0;
  for (uint8_t i = 0; i < 48; i++) {
    if (i < first || enabled == spad_count) map[i / 8] &= ~(1 << (i % 8));
    else if ((map[i / 8] >> (i % 8)) & 1) enabled++;
  }
  wrn(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6);

  for (unsigned i = 0; i < sizeof(TUNING) / sizeof(TUNING[0]); i++) wr(d, TUNING[i][0], TUNING[i][1]);

  /* ngắt GPIO: có mẫu mới, mức thấp tích cực */
  wr(d, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  wr(d, GPIO_HV_MUX_ACTIVE_HIGH, rd(d, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
  wr(d, SYSTEM_INTERRUPT_CLEAR, 0x01);

  d->budget_us = get_budget(d);
  wr(d, SYSTEM_SEQUENCE_CONFIG, 0xE8); /* tắt MSRC và TCC */
  vl53l0x_set_timing_budget(d, d->budget_us);

  wr(d, SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!single_ref_cal(d, 0x40)) return false; /* VHV */
  wr(d, SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!single_ref_cal(d, 0x00)) return false; /* phase */
  wr(d, SYSTEM_SEQUENCE_CONFIG, 0xE8);
  return d->ok;
}

bool vl53l0x_start_continuous(vl53l0x_t *d)
{
  wr(d, 0x80, 0x01); wr(d, 0xFF, 0x01); wr(d, 0x00, 0x00);
  wr(d, 0x91, d->stop_variable);
  wr(d, 0x00, 0x01); wr(d, 0xFF, 0x00); wr(d, 0x80, 0x00);
  wr(d, SYSRANGE_START, 0x02); /* back-to-back */
  return d->ok;
}

bool vl53l0x_poll(vl53l0x_t *d, uint16_t *mm, uint8_t *status)
{
  if ((rd(d, RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  uint8_t b[12];
  rdn(d, RESULT_RANGE_STATUS, b, 12);
  *status = (b[0] >> 3) & 0x0F;
  *mm = (uint16_t)(b[10] << 8 | b[11]);
  wr(d, SYSTEM_INTERRUPT_CLEAR, 0x01);
  return true;
}
