/* Đếm thời gian độ phân giải cao bằng bộ đếm chu kỳ CPU (DWT->CYCCNT).
 * Tràn sau ~43 s ở 100 MHz; so sánh bằng phép trừ không dấu nên vẫn đúng qua điểm tràn. */
#ifndef TIMEBASE_H
#define TIMEBASE_H

#include "main.h"

#define CYC_PER_MS (SystemCoreClock / 1000U)

void timebase_init(void);

static inline uint32_t cyc_now(void) { return DWT->CYCCNT; }

#endif /* TIMEBASE_H */
