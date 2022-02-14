#pragma once

#include "ffb_descriptor.h"

#include <stdint.h>

typedef struct {
    uint8_t friction;
    uint8_t spring;
    uint8_t damper;
    uint8_t inertia;
} effect_gain_t;
void effect_gain_t_init(effect_gain_t* self);

typedef struct {
    int effects_active; // was ffb_active
    uint8_t global_gain;
    float damper_f, damper_q;
    float friction_f, friction_q; //50 0.2
    float inertia_f, inertia_q;
    uint32_t calcfrequency; // HID frequency 1khz
    uint32_t cfFilter_f; // 500 = off
    uint8_t cfFilter_q; // User settable. q * 10
    float cfFilter_qfloatScaler;

// Rescale factor for conditional effect to boost or decrease the intensity
    float spring_scaler;
    float friction_scaler;
    float damper_scaler;
    float inertia_scaler;
    int frictionPctSpeedToRampup;                                        // define the max value of the range (0..5% of maxspeed) where torque is rampup on friction
    float speedRampupPct;
    effect_gain_t gain;

    uint32_t effects_used;
} EffectsCalculator;
void EffectsCalculator_init(EffectsCalculator* self);
void EffectsCalculator_logEffectType(EffectsCalculator* self, uint8_t type);
void EffectsCalculator_setFilters(EffectsCalculator* self, FFB_Effect *effect);
void EffectsCalculator_setGain(EffectsCalculator* self, uint8_t gain);
void EffectsCalculator_calculate_ffb_effect(EffectsCalculator* self, FFB_Effect * effects);
