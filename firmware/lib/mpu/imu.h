/* Ước lượng góc từ MPU6050 trên I2C2 (SCL PB10, SDA PB3).
 *   - yaw: tích phân gyro z (hình thang), đã trừ bias và nhân hệ số tỉ lệ
 *   - roll/pitch: bộ lọc bù gyro + accel
 *   - tự bù trôi: khi xe đứng yên (moving = 0 và gz nhỏ trong 300 ms) thì cập nhật bias gz và ngừng cộng yaw
 *   - tự gỡ treo bus I2C khi đọc lỗi liên tiếp (nhiễu động cơ làm MPU giữ SDA thấp)
 * Quy ước: yaw dương = xe quay ngược chiều kim đồng hồ nhìn từ trên (quay trái). */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
  float yaw, roll, pitch; /* độ */
  float gz;               /* tốc độ quay quanh trục đứng, đã lọc (độ/s) */
} imu_att_t;

typedef void (*imu_progress_cb)(int pct);

uint8_t imu_init(void); /* trả về WHO_AM_I (0x68 chuẩn, bản clone 0x70...), 0 = không trả lời */
int imu_ok(void);

/* Đo bias gyro trong ms mili giây (xe phải nằm yên, hàm chặn). Trả về 1 nếu thành công,
 * noise_out = độ lệch chuẩn gz (độ/s). Sau đó góc được đưa về 0. */
int imu_calibrate(uint32_t ms, imu_progress_cb cb, float *noise_out);

void imu_step(int moving); /* gọi mỗi 1 ms; moving = đang có lệnh động cơ hoặc encoder vừa đổi */

void imu_zero_yaw(void);
void imu_set_scale(float k); /* hệ số tỉ lệ gyro z (đo bằng cách xoay xe N vòng) */
const imu_att_t *imu_att(void);
void imu_get_bias(float bias[3]);
int imu_still(void);              /* 1 = đang đứng yên, đang tự bù trôi */
int imu_live(void);               /* 1 = có lần đọc thành công trong 200 ms gần nhất */
uint32_t imu_recover_count(void); /* số lần đã gỡ treo bus */
void imu_bus_recover(void);       /* gỡ treo bus I2C2 thủ công */

#endif /* IMU_H */
