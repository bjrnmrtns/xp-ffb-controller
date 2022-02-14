#include "ffb_effects_calc.h"
#include "ffb_effects.h"
#include "ffb_math.h"

#include "biquad.h"

#include "stm32f3xx_hal.h"
#include <stddef.h>
#include <stdint.h>
#define X_AXIS_ENABLE 1
#define Y_AXIS_ENABLE 2
#define Z_AXIS_ENABLE 4
#define DIRECTION_ENABLE (1 << MAX_AXIS)

#define EFFECT_STATE_INACTIVE 0

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

void EffectsCalculator_setGain(EffectsCalculator* self, uint8_t gain)
{
    self->global_gain = gain;
}

void EffectsCalculator_calculate_ffb_effect(EffectsCalculator* self, FFB_Effect * effects)
{
//    for (auto &axis : axes) {
//        axis->calculateAxisEffects(isActive());
//    }

    if(!self->effects_active){
        return;
    }

    int32_t forceX = 0;
    int32_t forceY = 0;
    int32_t forceVector = 0;
    uint8_t axisCount = 1;
    int validY = axisCount > 1;
#if MAX_AXIS == 3
    int32_t forceZ = 0;
	bool validZ = axisCount > 2;
#endif

    for (uint8_t i = 0; i < MAX_EFFECTS; i++)
    {
        FFB_Effect *effect = &effects[i];

        // Effect activated and not infinite
        if (effect->state != EFFECT_STATE_INACTIVE && effect->duration != FFB_EFFECT_DURATION_INFINITE){
            // Start delay not yet reached
            if(HAL_GetTick() < effect->startTime){
                continue;
            }
            // If effect has expired make inactive
            if (HAL_GetTick() > effect->startTime + effect->duration)
            {
                effect->state = EFFECT_STATE_INACTIVE;
            }
        }

        // Filter out inactive effects
        if (effect->state == EFFECT_STATE_INACTIVE)
        {
            continue;
        }


        if (effect->conditionsCount == 0) {
         //   forceVector = calcNonConditionEffectForce(effect);
        }

        if (effect->enableAxis == DIRECTION_ENABLE || (effect->enableAxis & X_AXIS_ENABLE))
        {
        //    forceX += calcComponentForce(effect, forceVector, axes, 0);
            forceX = clip_i(forceX, -0x7fff, 0x7fff); // Clip
        }
        if (validY && ((effect->enableAxis == DIRECTION_ENABLE) || (effect->enableAxis & Y_AXIS_ENABLE)))
        {
         //   forceY += calcComponentForce(effect, forceVector, axes, 1);
            forceY = clip_i(forceY, -0x7fff, 0x7fff); // Clip
        }

    }

    //axes[0]->setEffectTorque(forceX);
    if (validY)
    {
        //axes[1]->setEffectTorque(forceY);
    }
}
