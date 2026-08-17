#ifndef __USER_FOC_MATH_H__
#define __USER_FOC_MATH_H__

#include <stdint.h>

#define FOC_Q10_ONE             (1024)
#define _PI_2_1000              (1570)
#define _PI_1000                (3141)
#define _3PI_2_1000             (4712)
#define _2PI_1000               (6283)
#define _PI_3_1000              (1047)
#define ONE_OVER_SQRT3_Q10      (591)
#define SQRT3_Q10               (1773)

extern const int16_t sine_array[1572];

int32_t _sin_q10(uint16_t a);
int32_t _cos_q10(uint16_t a);
uint16_t _normalizeAngle(int32_t angle);
uint16_t FOC_AngleNormalize(uint16_t angle);
int32_t _sqrt_fast(int32_t x);
int32_t fast_atan2_int(int32_t y, int32_t x);
int32_t FOC_MulQ10(int32_t a, int32_t b);

#endif
