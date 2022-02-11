#include "biquad.h"

#include <math.h>
#define CLIP(x, upper, lower) (fmin(upper, fmax(x, lower)))

void biquad_init_default(struct Biquad* self) {
    self->z1 = self->z2 = 0.0;
}
void biquad_init(struct Biquad* self, enum BiquadType type, float Fc, float Q, float peakGain) {
    setBiquad(self, type, Fc, Q, peakGain);
}

float biquad_process(struct Biquad* self, float in) {
    float out = in * self->a0 + self->z1;
    self->z1 = in * self->a1 + self->z2 - self->b1 * out;
    self->z2 = in * self->a2 - self->b2 * out;
    return out;
}

void setBiquad(struct Biquad* self, enum BiquadType type, float Fc, float Q, float peakGain) {
    self->Fc = CLIP(Fc,0,0.5);
    self->type = type;
    self->Q = Q;
    self->Fc = Fc;
    self->peakGain = peakGain;
    calcBiquad(&self);
}

void setFc(struct Biquad* self, float Fc) {
    Fc = CLIP(Fc,0,0.5);
    self->Fc = Fc;
    calcBiquad(self);
}

void setQ(struct Biquad* self, float Q) {
    self->Q = Q;
    calcBiquad(self);
}

void calcBiquad(struct Biquad* self) {
    self->z1 = 0.0;
    self->z2 = 0.0;
    float norm;
    float V = pow(10, fabs(self->peakGain) / 20.0);
    float K = tan(M_PI * self->Fc);
    switch (self->type) {
        case lowpass:
            norm = 1 / (1 + K / self->Q + K * K);
            self->a0 = K * K * norm;
            self->a1 = 2 * self->a0;
            self->a2 = self->a0;
            self->b1 = 2 * (K * K - 1) * norm;
            self->b2 = (1 - K / self->Q + K * K) * norm;
            break;

        case highpass:
            norm = 1 / (1 + K / self->Q + K * K);
            self->a0 = 1 * norm;
            self->a1 = -2 * self->a0;
            self->a2 = self->a0;
            self->b1 = 2 * (K * K - 1) * norm;
            self->b2 = (1 - K / self->Q + K * K) * norm;
            break;

        case bandpass:
            norm = 1 / (1 + K / self->Q + K * K);
            self->a0 = K / self->Q * norm;
            self->a1 = 0;
            self->a2 = -self->a0;
            self->b1 = 2 * (K * K - 1) * norm;
            self->b2 = (1 - K / self->Q + K * K) * norm;
            break;

        case notch:
            norm = 1 / (1 + K / self->Q + K * K);
            self->a0 = (1 + K * K) * norm;
            self->a1 = 2 * (K * K - 1) * norm;
            self->a2 = self->a0;
            self->b1 = self->a1;
            self->b2 = (1 - K / self->Q + K * K) * norm;
            break;

        case peak:
            if (self->peakGain >= 0) {    // boost
                norm = 1 / (1 + 1/self->Q * K + K * K);
                self->a0 = (1 + V/self->Q * K + K * K) * norm;
                self->a1 = 2 * (K * K - 1) * norm;
                self->a2 = (1 - V/self->Q * K + K * K) * norm;
                self->b1 = self->a1;
                self->b2 = (1 - 1/self->Q * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (1 + V/self->Q * K + K * K);
                self->a0 = (1 + 1/self->Q * K + K * K) * norm;
                self->a1 = 2 * (K * K - 1) * norm;
                self->a2 = (1 - 1/self->Q * K + K * K) * norm;
                self->b1 = self->a1;
                self->b2 = (1 - V/self->Q * K + K * K) * norm;
            }
            break;
        case lowshelf:
            if (self->peakGain >= 0) {    // boost
                norm = 1 / (1 + sqrt(2) * K + K * K);
                self->a0 = (1 + sqrt(2*V) * K + V * K * K) * norm;
                self->a1 = 2 * (V * K * K - 1) * norm;
                self->a2 = (1 - sqrt(2*V) * K + V * K * K) * norm;
                self->b1 = 2 * (K * K - 1) * norm;
                self->b2 = (1 - sqrt(2) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (1 + sqrt(2*V) * K + V * K * K);
                self->a0 = (1 + sqrt(2) * K + K * K) * norm;
                self->a1 = 2 * (K * K - 1) * norm;
                self->a2 = (1 - sqrt(2) * K + K * K) * norm;
                self->b1 = 2 * (V * K * K - 1) * norm;
                self->b2 = (1 - sqrt(2*V) * K + V * K * K) * norm;
            }
            break;
        case highshelf:
            if (self->peakGain >= 0) {    // boost
                norm = 1 / (1 + sqrt(2) * K + K * K);
                self->a0 = (V + sqrt(2*V) * K + K * K) * norm;
                self->a1 = 2 * (K * K - V) * norm;
                self->a2 = (V - sqrt(2*V) * K + K * K) * norm;
                self->b1 = 2 * (K * K - 1) * norm;
                self->b2 = (1 - sqrt(2) * K + K * K) * norm;
            }
            else {    // cut
                norm = 1 / (V + sqrt(2*V) * K + K * K);
                self->a0 = (1 + sqrt(2) * K + K * K) * norm;
                self->a1 = 2 * (K * K - 1) * norm;
                self->a2 = (1 - sqrt(2) * K + K * K) * norm;
                self->b1 = 2 * (K * K - V) * norm;
                self->b2 = (V - sqrt(2*V) * K + K * K) * norm;
            }
            break;
    }
}
