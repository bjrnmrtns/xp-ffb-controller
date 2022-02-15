#pragma once

#include "stdint.h"

enum class BiquadType : uint8_t {
    lowpass = 0,
    highpass,
    bandpass,
    notch,
    peak,
    lowshelf,
    highshelf
};

struct Biquad {
    Biquad(BiquadType type, float Fc, float Q, float peakGain);
    float process(float in);
    void setFc(float Fc);
    void setQ(float Q);
    void calcBiquad();
    BiquadType type;
    float a0, a1, a2, b1, b2;
    float Fc, Q, peakGain;
    float z1, z2;
};