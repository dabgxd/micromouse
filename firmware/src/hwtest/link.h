/* Đường truyền UART2 với web: nhận bằng DMA vòng, gửi bằng DMA (không chặn vòng điều khiển). */
#ifndef LINK_H
#define LINK_H

void link_init(void);
void link_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2))); /* đưa vào hàng đợi gửi */
void link_flush(void);                       /* đẩy hàng đợi sang DMA nếu DMA đang rảnh */
int link_read_line(char *line, int max);     /* 1 = có một dòng lệnh hoàn chỉnh */

#endif /* LINK_H */
