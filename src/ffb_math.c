#include "ffb_math.h"

#include <math.h>
#include <stdint.h>

uint32_t min(uint32_t x, uint32_t y) {
    return x < y ? x : y;
}

uint32_t max(uint32_t x, uint32_t y) {
    return x > y ? x : y;
}

float clip_f(float val, float upper, float lower)
{
    return fmin(upper, fmax(val, lower));
}

float clip_u(uint32_t val, uint32_t upper, uint32_t lower)
{
    return min(upper, max(val, lower));
}

float clip_i(int32_t val, int32_t upper, int32_t lower)
{
    return min(upper, max(val, lower));
}
