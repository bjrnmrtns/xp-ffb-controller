#pragma once

#include "stdint.h"
typedef enum {
    lowpass = 0,
    highpass,
    bandpass,
    notch,
    peak,
    lowshelf,
    highshelf
} BiquadType;

typedef struct {
    BiquadType type;
    float a0, a1, a2, b1, b2;
    float Fc, Q, peakGain;
    float z1, z2;
} Biquad;

void biquad_init(Biquad* self, BiquadType type, float Fc, float Q, float peakGainDB);
void biquad_deinit(Biquad* self);
float biquad_process(Biquad* self, float in);
void setBiquad(Biquad* self, BiquadType type, float Fc, float Q, float peakGain);
void setFc(Biquad* self, float Fc); //frequency
void setQ(Biquad* self, float Q);
void calcBiquad(Biquad* self);
