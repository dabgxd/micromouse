#include "link.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"

extern UART_HandleTypeDef huart2;

static uint8_t rxbuf[256];
static uint32_t rx_tail;
static char txq[1024], txdma[1024];
static uint16_t txq_len;

void link_init(void) { HAL_UART_Receive_DMA(&huart2, rxbuf, sizeof rxbuf); }

void link_flush(void)
{
  if (!txq_len || huart2.gState != HAL_UART_STATE_READY) return;
  memcpy(txdma, txq, txq_len);
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)txdma, txq_len);
  txq_len = 0;
}

void link_printf(const char *fmt, ...)
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
  link_flush();
}

static int rx_getc(void)
{
  uint32_t head = sizeof rxbuf - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
  if (head == rx_tail) return -1;
  uint8_t c = rxbuf[rx_tail];
  rx_tail = (rx_tail + 1) % sizeof rxbuf;
  return c;
}

int link_read_line(char *line, int max)
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
