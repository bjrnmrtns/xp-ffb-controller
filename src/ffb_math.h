#pragma once

#include <stdint.h>

float clip_f(float val, float upper, float lower);
float clip_u(uint32_t val, uint32_t upper, uint32_t lower);
float clip_i(int32_t val, int32_t upper, int32_t lower);
