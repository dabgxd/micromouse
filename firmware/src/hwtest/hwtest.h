/* Firmware test phần cứng, điều khiển bằng trang web tools/hwtest_web (Web Serial qua USB-TTL hoặc HC-05).
 * Chỉ build trong môi trường hwtest (chạy trong thư mục firmware/):  pio run -e hwtest -t upload
 * Dùng các thư viện trong lib/ (tof, mpu, tb6612, encoder, pid, motion), giống hệt code chính sẽ dùng.
 *
 * Giao thức (UART2 38400 8N1): mỗi dòng ASCII kết thúc bằng '\n'.
 *
 * Web -> xe
 *   ping | scan | stop
 *   tof init | tof on | tof off
 *   mpu init | mpu cal | mpu zero | mpu on | mpu off | mpu scale <k>
 *   m <L%> <R%>                         lái tay; phải gửi lại mỗi < 500 ms
 *   enc zero
 *   go <0=L|1=R|2=cả hai> <xung> <pct>  chạy hở N xung rồi phanh (để so sánh với PID)
 *   mv <xungL> <xungR> <vmax xung/s> <gia tốc xung/s²>
 *   vel <xung/s L> <xung/s R> <gia tốc>  phải gửi lại mỗi < 600 ms
 *   st <xung> <vmax> <gia tốc> <kh> <kg> <góc lệch> <kc>
 *   rot <độ, + = trái> <w max độ/s> <gia tốc độ/s²> <kt> <ktd> <xung/độ>
 *   pid <kp> <ki> <kd>
 *   ff <deadL> <kvL> <deadR> <kvR> [<khởi động L> <khởi động R>]
 *   ident                               đo đặc tính động cơ (kê bánh lên)
 *
 * Xe -> web (sự kiện)
 *   HELLO | PONG | LOG,<chữ>
 *   I2C,<bus>,<địa chỉ hex>...
 *   TOFI,<i>,<địa chỉ hex>,<ok>
 *   MPUI,<who hex> | CALP,<%> | CALD,<bias x>,<bias y>,<bias z>,<nhiễu gz>
 *   GOD,<bánh>,<xung>,<ms>,<hết giờ>
 *   MVD,<bánh>,<sai cuối>,<sai max>,<ms>,<hết giờ> | VLD
 *   STD,<dL>,<dR>,<sai hướng cuối>,<sai hướng max>,<ms>,<gyro>
 *   RTD,<sai cuối>,<góc>,<dL>,<dR>,<ms>,<gyro>,<ms ổn định>
 *   IDS,<khởi động L>,<khởi động R> | IDN,<pwm>,<xung/s L>,<xung/s R> | IDD
 *
 * Xe -> web (telemetry 25 Hz)
 *   ENC,<xung L>,<xung R>,<xung/s L>,<xung/s R>              luôn gửi
 *   TOF,<mm L90>,<L45>,<M0>,<R45>,<R90>                      sau "tof on" (-1 = ngoài tầm)
 *   IMU,<yaw>,<roll>,<pitch>,<gz>,<live>,<số lần gỡ bus>,<đứng yên>,<bias gz>   sau "mpu on"
 *   MVT,<saiL>,<saiR>,<uL>,<uR>                              khi đang mv
 *   VLT,<v đặt L>,<v L>,<uL>,<saiL>,<v đặt R>,<v R>,<uR>,<saiR>   khi đang vel
 *   STT,<s đặt>,<dL>,<dR>,<sai hướng>,<dv>,<uL>,<uR>,<v đặt>,<saiL>,<saiR>   khi đang st
 *   RTT,<góc đặt>,<góc>,<gz>,<dw>,<dL>,<dR>,<uL>,<uR>,<saiL>,<saiR>         khi đang rot
 * Đơn vị: xung encoder, xung/s, %, độ, độ/s, ms. */
#ifndef HWTEST_H
#define HWTEST_H

void hwtest_run(void); /* không bao giờ trả về */

#endif /* HWTEST_H */
