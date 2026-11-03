/* Encoder 2 bánh, đếm x4 bằng timer phần cứng:
 *   bánh TRÁI = TIM3 (A = PA7, B = PA6), bánh PHẢI = TIM4 (A = PB7, B = PB6)
 * Bộ đếm 16 bit được cộng dồn thành 32 bit, nên phải gọi encoder_update() thường xuyên
 * (1 kHz là dư: tràn 16 bit cần > 20 vòng bánh giữa 2 lần gọi).
 * Encoder phải lắp đối xứng nên được đảo dấu: xe chạy tiến = số xung tăng ở cả hai bánh. */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_init(void);         /* bật 2 timer encoder và đưa bộ đếm về 0 */
void encoder_update(void);       /* đọc timer, cộng dồn phần thay đổi */
int32_t encoder_count(int side); /* số xung tích luỹ, side = SIDE_L / SIDE_R */
void encoder_zero(void);

#endif /* ENCODER_H */
