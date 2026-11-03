/* Driver TB6612FNG cho 2 động cơ N20.
 *   Kênh A = bánh PHẢI: PWMA PA8 (TIM1_CH1), AIN1 PB12, AIN2 PB13
 *   Kênh B = bánh TRÁI: PWMB PA9 (TIM1_CH2), BIN1 PB14, BIN2 PB15
 *   STBY PA10
 * Động cơ phải lắp đối xứng nên được đảo chiều bên trong: pct dương = cả hai bánh chạy TIẾN. */
#ifndef TB6612_H
#define TB6612_H

void tb6612_init(void);               /* bật PWM, nhả STBY, thả trôi cả hai động cơ */
void tb6612_set(int side, float pct); /* -100..100 % PWM, side = SIDE_L / SIDE_R */
void tb6612_brake(int side);          /* phanh ngắn mạch (IN1 = IN2 = 1) */
void tb6612_stop_all(void);           /* PWM = 0, thả trôi cả hai */

#endif /* TB6612_H */
