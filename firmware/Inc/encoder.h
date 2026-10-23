#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

/* Configure TIM3/TIM4 for x4 quadrature counting and start both encoders. */
bool Encoder_Init(void);

/* Return signed count deltas since the previous call, with 16-bit wrap handled. */
void Encoder_GetDelta(int32_t *left_delta, int32_t *right_delta);

/* Clear both hardware counters and the saved delta baseline. */
void Encoder_Reset(void);

#endif /* ENCODER_H_ */
