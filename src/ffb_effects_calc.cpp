#include "ffb_effects_calc.h"
#include "ffb_effects.h"

#include "biquad.h"

#include "stm32f3xx_hal.h"
#include <algorithm>
#include <math.h>
#include <stdlib.h>
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
                effect->filter[i] = Biquad(BiquadType::lowpass, self->damper_f / (float)self->calcfrequency, self->damper_q,0.0);
            }
            break;
        case FFB_EFFECT_FRICTION:
            for(size_t i = 0; i < MAX_AXIS; i++) {
                    effect->filter[i] = Biquad(BiquadType::lowpass, self->friction_f / (float)self->calcfrequency, self->friction_q, 0.0);
            }
            break;
        case FFB_EFFECT_INERTIA:
            for(size_t i = 0; i < MAX_AXIS; i++) {
                    effect->filter[i] = Biquad(BiquadType::lowpass, self->inertia_f / (float)self->calcfrequency, self->inertia_q, 0.0);
            }
            break;
        case FFB_EFFECT_CONSTANT:
            for(size_t i = 0; i < MAX_AXIS; i++) {
                    effect->filter[i] = Biquad(BiquadType::lowpass, self->cfFilter_f / (float)self->calcfrequency,
                                      self->cfFilter_qfloatScaler * (self->cfFilter_q + 1), 0.0);
            }
            break;
    }
}

void EffectsCalculator_setGain(EffectsCalculator* self, uint8_t gain)
{
    self->global_gain = gain;
}

void EffectsCalculator_calculate_ffb_effect(EffectsCalculator* self, std::array<FFB_Effect, MAX_EFFECTS>& effects)
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
            forceVector = EffectsCalculator_calcNonConditionEffectForce(self, effect);
        }

        if (effect->enableAxis == DIRECTION_ENABLE || (effect->enableAxis & X_AXIS_ENABLE))
        {
            forceX += EffectsCalculator_calcComponentForce(self, effect, forceVector, 0);
            forceX = std::clamp(forceX, (int32_t)-0x7fff, (int32_t)0x7fff); // Clip
        }
        if (validY && ((effect->enableAxis == DIRECTION_ENABLE) || (effect->enableAxis & Y_AXIS_ENABLE)))
        {
            forceY += EffectsCalculator_calcComponentForce(self, effect, forceVector, 1);
            forceY = std::clamp(forceY, (int32_t)-0x7fff, (int32_t)0x7fff); // Clip
        }

    }

    //axes[0]->setEffectTorque(forceX);
    if (validY)
    {
        //axes[1]->setEffectTorque(forceY);
    }
}

int32_t EffectsCalculator_applyEnvelope(FFB_Effect *effect, int32_t value)
{
    int32_t magnitude = (effect->magnitude);
    int32_t attackLevel = (effect->attackLevel);
    int32_t fadeLevel = (effect->fadeLevel);
    int32_t newValue = magnitude;
    uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
    if (elapsed_time < effect->attackTime)
    {
        newValue = (magnitude - attackLevel) * elapsed_time;
        newValue /= (int32_t)effect->attackTime;
        newValue += attackLevel;
    }
    if (effect->duration != FFB_EFFECT_DURATION_INFINITE &&
        elapsed_time > (effect->duration - effect->fadeTime))
    {
        newValue = (magnitude - fadeLevel) * (effect->duration - elapsed_time);
        newValue /= (int32_t)effect->fadeTime;
        newValue += fadeLevel;
    }

    newValue *= value;
    newValue /= 0x7fff; // 16 bit
    return newValue;
}

int32_t EffectsCalculator_calcNonConditionEffectForce(EffectsCalculator* self, FFB_Effect *effect) {
    int32_t force_vector = 0;
    switch (effect->type){

        case FFB_EFFECT_CONSTANT:
        { // Constant force is just the force
            force_vector = ((int32_t)effect->magnitude * (int32_t)(1 + effect->gain)) >> 8;
            // Optional filtering to reduce spikes
            if (self->cfFilter_f < self->calcfrequency / 2 && self->cfFilter_f != 0 )
            {
                force_vector = effect->filter[0].value().process(force_vector);
            }
            break;
        }

        case FFB_EFFECT_RAMP:
        {
            uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
            int32_t duration = effect->duration;
            float force = (int32_t)effect->startLevel + ((int32_t)elapsed_time * (effect->endLevel - effect->startLevel)) / duration;
            force_vector = (int32_t)(force * (1 + effect->gain)) >> 8;
            break;
        }

        case FFB_EFFECT_SQUARE:
        {
            uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
            int32_t force = ((elapsed_time + effect->phase) % ((uint32_t)effect->period + 2)) < (uint32_t)(effect->period + 2) / 2 ? -effect->magnitude : effect->magnitude;
            force_vector = force + effect->offset;
            break;
        }

        case FFB_EFFECT_TRIANGLE:
        {
            int32_t force = 0;
            int32_t offset = effect->offset;
            int32_t magnitude = effect->magnitude;
            uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
            uint32_t phase = effect->phase;
            uint32_t period = effect->period;
            float periodF = period;

            int32_t maxMagnitude = offset + magnitude;
            int32_t minMagnitude = offset - magnitude;
            uint32_t phasetime = (phase * period) / 35999;
            uint32_t timeTemp = elapsed_time + phasetime;
            float remainder = timeTemp % period;
            float slope = ((maxMagnitude - minMagnitude) * 2) / periodF;
            if (remainder > (periodF / 2))
                force = slope * (periodF - remainder);
            else
                force = slope * remainder;
            force += minMagnitude;
            force_vector = force;
            break;
        }

        case FFB_EFFECT_SAWTOOTHUP:
        {
            float offset = effect->offset;
            float magnitude = effect->magnitude;
            uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
            uint32_t phase = effect->phase;
            uint32_t period = effect->period;
            float periodF = effect->period;

            float maxMagnitude = offset + magnitude;
            float minMagnitude = offset - magnitude;
            int32_t phasetime = (phase * period) / 35999;
            uint32_t timeTemp = elapsed_time + phasetime;
            float remainder = timeTemp % period;
            float slope = (maxMagnitude - minMagnitude) / periodF;
            force_vector = (int32_t)(minMagnitude + slope * (period - remainder));
            break;
        }

        case FFB_EFFECT_SAWTOOTHDOWN:
        {
            float offset = effect->offset;
            float magnitude = effect->magnitude;
            uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
            float phase = effect->phase;
            uint32_t period = effect->period;
            float periodF = effect->period;

            float maxMagnitude = offset + magnitude;
            float minMagnitude = offset - magnitude;
            int32_t phasetime = (phase * period) / 35999;
            uint32_t timeTemp = elapsed_time + phasetime;
            float remainder = timeTemp % period;
            float slope = (maxMagnitude - minMagnitude) / periodF;
            force_vector = (int32_t)(minMagnitude + slope * (remainder)); // reverse time
            break;
        }

        case FFB_EFFECT_SINE:
        {
            uint32_t t = HAL_GetTick() - effect->startTime;
            float freq = 1.0f / fmax(effect->period, 2);
            float phase = (float)effect->phase / (float)35999; //degrees
            float sine = sinf(2.0 * (float)M_PI * (t * freq + phase)) * effect->magnitude;
            force_vector = (int32_t)(effect->offset + sine);
            break;
        }
        default:
            break;
    }
    if(effect->useEnvelope) {
        force_vector = EffectsCalculator_applyEnvelope(effect, (int32_t)force_vector);
    }
    return force_vector;
}

/**
 * Calculates a conditional effect
 * Takes care of deadband and offsets and scalers
 * Gain of 255 = 1x. Prescale with scale factor
 */
int32_t EffectsCalculator_calcConditionEffectForce(FFB_Effect *effect, float  metric, uint8_t gain,
                                                    uint8_t idx, float scale, float angle_ratio)
{
    int16_t offset = effect->conditions[idx].cpOffset;
    int16_t deadBand = effect->conditions[idx].deadBand;
    int32_t force = 0;
    float gainfactor = (float)(gain+1) / 256.0;

    // Effect is only active outside deadband + offset
    if (abs(metric - offset) > deadBand){
        float coefficient = effect->conditions[idx].negativeCoefficient;
        if(metric > offset){
            coefficient = effect->conditions[idx].positiveCoefficient;
        }
        coefficient /= 0x7fff; // rescale the coefficient of effect

        // remove offset/deadband from metric to compute force
        metric = metric - (offset + (deadBand * (metric < offset ? -1 : 1)) );

        force = std::clamp((coefficient * gainfactor * scale * (float)(metric)),
                           (float)-effect->conditions[idx].negativeSaturation,
                           (float)effect->conditions[idx].positiveSaturation);
    }


    return force * angle_ratio;
}

/*
 * If the number of Condition report blocks is equal to the number of axes for the effect, then the first report
block applies to the first axis, the second applies to the second axis, and so on. For example, a two-axis
spring condition with CP Offset set to zero in both Condition report blocks would have the same effect as
the joystick self-centering spring. When a condition is defined for each axis in this way, the effect must
not be rotated.

If there is a single Condition report block for an effect with more than one axis, then the direction along
which the parameters of the Condition report block are in effect is determined by the direction parameters
passed in the Direction field of the Effect report block. For example, a friction condition rotated 45
degrees (in polar coordinates) would resist joystick motion in the northeast-southwest direction but would
have no effect on joystick motion in the northwest-southeast direction.
 */
int32_t EffectsCalculator_calcComponentForce(EffectsCalculator* self, FFB_Effect *effect, int32_t forceVector, /*std::vector<std::unique_ptr<Axis>> &axes,*/ uint8_t axis)
{
	int32_t result_torque = 0;
	uint16_t direction;
	uint8_t con_idx = 0; // condition block index

	metric_t *metrics;// = axes[axis]->getMetrics();
	uint8_t axisCount = 1;// = axes.size();
	float scaleSpeed = 40;//axes[axis]->getSpeedScalerNormalized(); // TODO decide if scalers are useful or not
	float scaleAccel = 40;//axes[axis]->getAccelScalerNormalized();

	if (effect->enableAxis == DIRECTION_ENABLE)
	{
		direction = effect->directionX;
		if (effect->conditionsCount > 1)
		{
			con_idx = axis;
		}
	}
	else
	{
		direction = axis == 0 ? effect->directionX : effect->directionY;
		con_idx = axis;
	}

	//bool useForceDirectionForConditionEffect = (effect->enableAxis == DIRECTION_ENABLE && axisCount > 1 && effect->conditionsCount == 1);
	bool rotateConditionForce = (axisCount > 1 && effect->conditionsCount < axisCount);
	float angle = ((float)direction * (2*M_PI) / 36000.0);
	float angle_ratio = axis == 0 ? sin(angle) : -1 * cos(angle);
	angle_ratio = rotateConditionForce ? angle_ratio : 1.0;

	switch (effect->type)
	{
	case FFB_EFFECT_CONSTANT:
	case FFB_EFFECT_RAMP:
	case FFB_EFFECT_SQUARE:
	case FFB_EFFECT_TRIANGLE:
	case FFB_EFFECT_SAWTOOTHUP:
	case FFB_EFFECT_SAWTOOTHDOWN:
	case FFB_EFFECT_SINE:
	{
		result_torque = -forceVector * angle_ratio;
		break;
	}

	case FFB_EFFECT_SPRING:
	{
		float pos = metrics->pos;
		result_torque -= EffectsCalculator_calcConditionEffectForce(effect, pos, self->gain.spring, con_idx, self->spring_scaler, angle_ratio);
		break;
	}


	/** 	      |	  (rampup is from 0..5% of max velocity)
	 * 			  |	  __________ (after use max coefficient)
	 * 			  |	 /
	 *			  |	/
	 *			  |-
	 * ------------------------  Velocity
	 * 			 -|
	 *			/ |
	 * 		   /  |
	 * 	-------   |
	 * 			  |
	 */
	case FFB_EFFECT_FRICTION: // TODO sometimes unstable.
	{
		float speed = metrics->speed * scaleSpeed;//effect->filter[con_idx]->process()

		int16_t offset = effect->conditions[con_idx].cpOffset;
		int16_t deadBand = effect->conditions[con_idx].deadBand;
		int32_t force = 0;

		// Effect is only active outside deadband + offset
		if (abs((int32_t)speed - offset) > deadBand){

			// remove offset/deadband from metric to compute force
			speed -= (offset + (deadBand * (speed < offset ? -1 : 1)) );

			// check if speed is in the 0..x% to rampup, if is this range, apply a sinusoidale function to smooth the torque (slow near 0, slow around the X% rampup
			float rampupFactor = 1.0;
			if (fabs (speed) < self->speedRampupPct) {								// if speed in the range to rampup we apply a sinus curbe to ramup

				float phaseRad = M_PI * ((fabs (speed) / self->speedRampupPct) - 0.5);// we start to compute the normalized angle (speed / normalizedSpeed@5%) and translate it of -1/2PI to translate sin on 1/2 periode
				rampupFactor = ( 1 + sin(phaseRad ) ) / 2;						// sin value is -1..1 range, we translate it to 0..2 and we scale it by 2

			}

			int8_t sign = speed >= 0 ? 1 : -1;
			uint16_t coeff = speed < 0 ? effect->conditions[con_idx].negativeCoefficient : effect->conditions[con_idx].positiveCoefficient;
			force = coeff * rampupFactor * sign;

			//if there is a saturation, used it to clip result
			if (effect->conditions[con_idx].negativeSaturation !=0 || effect->conditions[con_idx].positiveSaturation !=0) {
				force = std::clamp(force, (int32_t)-effect->conditions[con_idx].negativeSaturation, (int32_t)effect->conditions[con_idx].positiveSaturation);
			}

//			static int32_t last_force = 0;
//
//			// if there is 2 successive torque with a different direction, we ignore the first one to remove oscillation
//			if (last_force * force >=0) {
//				force = ((gain.friction + 1) * force) >> 7;
//				result_torque -=  force * angle_ratio;
//			}
//			last_force = force;
			result_torque -= effect->filter[con_idx].value().process((((self->gain.friction + 1) * force) >> 8) * angle_ratio * self->friction_scaler);
		}
//			float accel = metrics->accel * scaleAccel;
//			result_torque -= calcConditionEffectForce(effect, accel, gain.friction, con_idx, friction_scaler, angle_ratio);

		break;
	}
	case FFB_EFFECT_DAMPER:
	{

		float speed = metrics->speed * scaleSpeed;
		result_torque -=  effect->filter[con_idx].value().process(EffectsCalculator_calcConditionEffectForce(effect,speed, self->gain.damper, con_idx, self->damper_scaler, angle_ratio));

		break;
	}

	case FFB_EFFECT_INERTIA:
	{
		float accel = metrics->accel* scaleAccel;
		result_torque -= effect->filter[con_idx].value().process(EffectsCalculator_calcConditionEffectForce(effect, accel, self->gain.inertia, con_idx, self->inertia_scaler, angle_ratio)); // Bump *60 the inertia feedback

		break;
	}

	default:
		// Unsupported effect
		break;
	}
	return (result_torque * (self->global_gain+1)) >> 8; // Apply global gain
}