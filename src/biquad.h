#pragma once

#include "stdint.h"
enum BiquadType {
    lowpass = 0,
    highpass,
    bandpass,
    notch,
    peak,
    lowshelf,
    highshelf
};

struct Biquad {
    enum BiquadType type;
    float a0, a1, a2, b1, b2;
    float Fc, Q, peakGain;
    float z1, z2;
};

void biquad_init(struct Biquad* self, enum BiquadType type, float Fc, float Q, float peakGainDB);
void biquad_deinit(struct Biquad* self);
float biquad_process(struct Biquad* self, float in);
void setBiquad(struct Biquad* self, enum BiquadType type, float Fc, float Q, float peakGain);
void setFc(struct Biquad* self, float Fc); //frequency
void setQ(struct Biquad* self, float Q);
void calcBiquad(struct Biquad* self);
