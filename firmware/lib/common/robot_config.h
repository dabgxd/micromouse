/* Thông số cơ khí và hằng số dùng chung cho firmware test (hwtest) và code chính.
 * Các giá trị đo trên xe thật bằng tools/hwtest_web; đo lại thì sửa ở đây. */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Chỉ số bánh / động cơ / encoder */
enum { SIDE_L = 0, SIDE_R = 1 };

#define ENC_CPR_WHEEL 1400.0f /* xung (đếm x4) trên 1 vòng bánh: 7 PPR x 4 x hộp số 50 */
#define WHEEL_CIRC_MM 140.0f  /* chu vi bánh, đo bằng thước (xe chạy 1 vòng bánh = 140 mm) */
#define TRACK_MM      68.0f   /* khoảng cách tâm 2 bánh theo bản vẽ; web ước lượng lại sau mỗi lần quay */
#define MM_PER_COUNT  (WHEEL_CIRC_MM / ENC_CPR_WHEEL)

#define CTRL_DT_S 0.001f /* chu kỳ vòng điều khiển (1 kHz) */

#endif /* ROBOT_CONFIG_H */
