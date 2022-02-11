#pragma once

#include <math.h>
#include <stdint.h>

float clip_f(float val, float upper, float lower);
float clip_u(uint32_t val, uint32_t upper, uint32_t lower);