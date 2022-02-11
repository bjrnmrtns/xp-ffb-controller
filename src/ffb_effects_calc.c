#include "ffb_effects_calc.h"

#include "biquad.h"

#include <stddef.h>

void effect_gain_t_init(effect_gain_t* self) {
    self->friction = 127;
    self->spring = 64;
    self->damper = 127;
    self->inertia = 127;
}

void EffectsCalculator_init(EffectsCalculator* self) {
    self->effects_active = 0; // was ffb_active
    self->global_gain = 0xff;
    self->damper_f = 30, self->damper_q = 0.4;
    self->friction_f = 50, self->friction_q = 0.2; //50 0.2
    self->inertia_f = 15, self->inertia_q = 0.2;
    self->calcfrequency = 1000; // HID frequency 1khz
    self->cfFilter_f = self->calcfrequency / 2; // 500 = off
    self->cfFilter_q = 70; // User settable. q * 10
    self->cfFilter_qfloatScaler = 0.01;

// Rescale factor for conditional effect to boost or decrease the intensity
    self->spring_scaler = 4.0f;
    self->friction_scaler = 0.4f;
    self->damper_scaler = 2.0f;
    self->inertia_scaler = 200.0f;
    self->frictionPctSpeedToRampup = 5;                                        // define the max value of the range (0..5% of maxspeed) where torque is rampup on friction
    self->speedRampupPct =
            (self->frictionPctSpeedToRampup / 100.0) * 32767;    // compute the normalizedSpeed of pctToRampup factor
    effect_gain_t_init(&self->gain);
    self->effects_used = 0;
}

void EffectsCalculator_logEffectType(EffectsCalculator* self, uint8_t type) {
    if(type > 0 && type < 32){
        self->effects_used |= 1<<(type-1);
    }
}

void EffectsCalculator_setFilters(EffectsCalculator* self, FFB_Effect *effect) {
    switch (effect->type)
    {
        case FFB_EFFECT_DAMPER:
            for(size_t i = 0; i < MAX_AXIS; i++) {
                biquad_init(&effect->filter[i], lowpass, self->damper_f / (float)self->calcfrequency, self->damper_q,0.0);
            }
            break;
        case FFB_EFFECT_FRICTION:
            for(size_t i = 0; i < MAX_AXIS; i++) {
                    biquad_init(&effect->filter[i], lowpass, self->friction_f / (float)self->calcfrequency, self->friction_q, 0.0);
            }
            break;
        case FFB_EFFECT_INERTIA:
            for(size_t i = 0; i < MAX_AXIS; i++) {
                    biquad_init(&effect->filter[i], lowpass, self->inertia_f / (float)self->calcfrequency, self->inertia_q, 0.0);
            }
            break;
        case FFB_EFFECT_CONSTANT:
            for(size_t i = 0; i < MAX_AXIS; i++) {
                    biquad_init(&effect->filter[i], lowpass, self->cfFilter_f / (float)self->calcfrequency,
                                      self->cfFilter_qfloatScaler * (self->cfFilter_q + 1), 0.0);
            }
            break;
    }
}
