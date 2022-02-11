#include "ffb_effects.h"
#include "ffb_descriptor.h"

#include <stdint.h>
#include "stm32f3xx_hal.h"
#include "tusb.h"

#define MAX_EFFECTS 40

extern UART_HandleTypeDef huart1;

hid_ffb_t hid_ffb;

void hid_ffb_t_init(hid_ffb_t* self) {
    self->hid_out_period = 0; // ms since last out report for measuring update rate

    self->last_effect_id = 0;
    self->used_effects = 0;
    self->ffb_active = 0;
    FFB_BlockLoad_Feature_Data_t_init(&self->blockLoad_report);
    FFB_PIDPool_Feature_Data_t_init(&self->pool_report);
    for(size_t i = 0; i < MAX_EFFECTS; i++) {
        FFB_Effect_init(&self->effects[i]);
    }
    EffectsCalculator_init(&self->effectsCalculator);
    reportFFB_status_t_init(&self->reportFFBStatus);
    self->lastOut = 0;

}


/*bool HID_SendReport(uint8_t *report, uint16_t len){
    return tud_hid_report(0, report, len);
}

void sendStatusReport(uint8_t effect){
    this->reportFFBStatus.effectBlockIndex = effect;
    this->reportFFBStatus.status = HID_ACTUATOR_POWER;
    if(this->ffb_active){
        this->reportFFBStatus.status |= HID_ENABLE_ACTUATORS;
        this->reportFFBStatus.status |= HID_EFFECT_PLAYING;
    }else{
        this->reportFFBStatus.status |= HID_EFFECT_PAUSE;
    }
    if(effect > 0 && effects[effect-1].state == 1)
        this->reportFFBStatus.status |= HID_EFFECT_PLAYING;
    //printf("Status: %d\n",reportFFBStatus.status);
    HID_SendReport(reinterpret_cast<uint8_t*>(&this->reportFFBStatus), sizeof(reportFFB_status_t));
}

void free_effect(uint16_t idx){
    if(idx < MAX_EFFECTS){
        effects[idx].type=FFB_EFFECT_NONE;
        for(int i=0; i< MAX_AXIS; i++) {
            if(effects[idx].filter[i] != nullptr){
                effects[idx].filter[i].reset(nullptr);
            }
        }
    }
}

void set_gain(uint8_t gain){
    assert(effects_calc != nullptr);
    effects_calc->setGain(gain);
}
*/
void set_filters(hid_ffb_t* self, FFB_Effect *effect){
    EffectsCalculator_setFilters(&self->effectsCalculator, effect);
}
/*

void set_constant_effect(FFB_SetConstantForce_Data_t* effect){
    effects[effect->effectBlockIndex-1].magnitude = effect->magnitude;
}*/
uint8_t find_free_effect(hid_ffb_t* self, uint8_t type){ //Will return the first effect index which is empty or the same type
    for(uint8_t i=0;i<MAX_EFFECTS;i++){
        if(self->effects[i].type == FFB_EFFECT_NONE){
            return i;
        }
    }
    return -1;
}

void new_effect(hid_ffb_t* self, FFB_CreateNewEffect_Feature_Data_t* effect){
    // Allocates a new effect

    uint8_t index = find_free_effect(self, effect->effectType); // next effect
    if(index == -1){
        self->blockLoad_report.loadStatus = 2;
        return;
    }
    FFB_Effect* new_effect = &self->effects[index];
    FFB_Effect_init(new_effect);
    new_effect->type = effect->effectType;
    EffectsCalculator_logEffectType(&self->effectsCalculator, effect->effectType);
    set_filters(self, new_effect);

    // Set block load report
    self->reportFFBStatus.effectBlockIndex = index + 1;
    self->blockLoad_report.effectBlockIndex = index + 1;
    self->used_effects++;
    self->blockLoad_report.ramPoolAvailable = MAX_EFFECTS-self->used_effects;
    self->blockLoad_report.loadStatus = 1;
}

/*
void set_effect(FFB_SetEffect_t* effect){
    uint8_t index = effect->effectBlockIndex;
    if(index > MAX_EFFECTS || index == 0)
        return;

    FFB_Effect* effect_p = &effects[index-1];

    if (effect_p->type != effect->effectType){
        effect_p->startTime = 0;
        set_filters(effect_p);
    }

    effect_p->gain = effect->gain;
    effect_p->type = effect->effectType;
    effect_p->samplePeriod = effect->samplePeriod;

    effect_p->enableAxis = effect->enableAxis;
    effect_p->directionX = effect->directionX;
    effect_p->directionY = effect->directionY;
#if MAX_AXIS == 3
    effect_p->directionZ = effect->directionZ;
#endif

    effect_p->duration = effect->duration;
    effect_p->startDelay = effect->startDelay;
    if(!ffb_active)
        start_FFB();
    //sendStatusReport(effect->effectBlockIndex);
}

void set_condition(FFB_SetCondition_Data_t *cond){
    uint8_t axis = cond->parameterBlockOffset;
    if (axis >= MAX_AXIS){
        return; // sanity check!
    }
    FFB_Effect *effect = &effects[cond->effectBlockIndex - 1];
    effect->conditions[axis].cpOffset = cond->cpOffset;
    effect->conditions[axis].negativeCoefficient = cond->negativeCoefficient;
    effect->conditions[axis].positiveCoefficient = cond->positiveCoefficient;
    effect->conditions[axis].negativeSaturation = cond->negativeSaturation;
    effect->conditions[axis].positiveSaturation = cond->positiveSaturation;
    effect->conditions[axis].deadBand = cond->deadBand;
    effect->conditionsCount++;
    if(effect->conditions[axis].positiveSaturation == 0){
        effect->conditions[axis].positiveSaturation = 0x7FFF;
    }
    if(effect->conditions[axis].negativeSaturation == 0){
        effect->conditions[axis].negativeSaturation = 0x7FFF;
    }
}

void set_envelope(FFB_SetEnvelope_Data_t *report){
    FFB_Effect *effect = &effects[report->effectBlockIndex - 1];

    effect->attackLevel = report->attackLevel;
    effect->attackTime = report->attackTime;
    effect->fadeLevel = report->fadeLevel;
    effect->fadeTime = report->fadeTime;
    effect->useEnvelope = true;
}
void set_ramp(FFB_SetRamp_Data_t *report){
    FFB_Effect *effect = &effects[report->effectBlockIndex - 1];
    effect->magnitude = 0x7fff; // Full magnitude for envelope calculation. This effect does not have a periodic report
    effect->startLevel = report->startLevel;
    effect->endLevel = report->endLevel;
}

void set_periodic(FFB_SetPeriodic_Data_t* report){
    FFB_Effect* effect = &effects[report->effectBlockIndex-1];

    effect->period = clip<uint32_t,uint32_t>(report->period,1,0x7fff); // Period is never 0
    effect->magnitude = report->magnitude;
    effect->offset = report->offset;
    effect->phase = report->phase;
    //effect->counter = 0;
}
*/
/*


void reset_ffb(){
    for(uint8_t i=0;i<MAX_EFFECTS;i++){
        free_effect(i);
    }
    this->reportFFBStatus.effectBlockIndex = 1;
    this->reportFFBStatus.status = (HID_ACTUATOR_POWER) | (HID_ENABLE_ACTUATORS);
    used_effects = 0;
}
*/

uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
    char buf[] = "hidGet\r\n";
    HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
    uint8_t id = report_id - FFB_ID_OFFSET;

    switch(id){
        case HID_ID_BLKLDREP:
            //printf("Get Block Report\n");
            memcpy(buffer,&hid_ffb.blockLoad_report,sizeof(FFB_BlockLoad_Feature_Data_t));
            return sizeof(FFB_BlockLoad_Feature_Data_t);
            break;
        case HID_ID_POOLREP:
            //printf("Get Pool Report\n");
            memcpy(buffer,&hid_ffb.pool_report,sizeof(FFB_PIDPool_Feature_Data_t));
            return sizeof(FFB_PIDPool_Feature_Data_t);
            break;
        default:
            break;
    }
    return 0;
}

void hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    char buf[] = "hidOut\r\n";
    HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
    // FFB Output Message
    const uint8_t* report = buffer;
    uint8_t event_idx = buffer[0] - FFB_ID_OFFSET;

    // -------- Out Reports --------
    switch(event_idx){
        case HID_ID_NEWEFREP: //add Effect Report. Feature
            new_effect(&hid_ffb, (FFB_CreateNewEffect_Feature_Data_t*)(report));
            break;
        case HID_ID_EFFREP: // Set Effect
            //set_effect((FFB_SetEffect_t*)(report));
            break;
        case HID_ID_CTRLREP: // Control report. 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
            //ffb_control(report[1]);
            //sendStatusReport(0);
            break;
        case HID_ID_GAINREP: // Set global gain
            //set_gain(report[1]);
            break;
        case HID_ID_ENVREP: // Envelope
            //set_envelope((FFB_SetEnvelope_Data_t *)report);
            break;
        case HID_ID_CONDREP: // Spring, Damper, Friction, Inertia
            //set_condition((FFB_SetCondition_Data_t*)report);
            break;
        case HID_ID_PRIDREP: // Periodic
            //set_periodic((FFB_SetPeriodic_Data_t*)report);
            break;
        case HID_ID_CONSTREP: // Constant
            //set_constant_effect((FFB_SetConstantForce_Data_t*)report);
            break;
        case HID_ID_RAMPREP: // Ramp
            //set_ramp((FFB_SetRamp_Data_t *)report);
            break;
        case HID_ID_CSTMREP: // Custom. pretty much never used
            //printf("Customrep");
            break;
        case HID_ID_SMPLREP: // Download sample
            //printf("Sampledl");
            break;
        case HID_ID_EFOPREP: //Effect operation
        {
            // Start or stop effect
            /*uint8_t id = report[1]-1;
            if(report[2] == 3){
                effects[id].state = 0; //Stop
                //printf("Stop %d\n",report[1]);
            }else{
                if(effects[id].state != 1){
                    set_filters(&effects[id]);
                    //effects[id].startTime = 0; // When an effect was stopped reset all parameters that could cause jerking
                }
                //printf("Start %d\n",report[1]);
                effects[id].startTime = HAL_GetTick() + effects[id].startDelay; // + effects[id].startDelay;
                effects[id].state = 1; //Start
            }
            //sendStatusReport(report[1]);
             */
            break;
        }
        case HID_ID_BLKFRREP: // Free a block
        {
            //free_effect(report[1]-1);
            break;
        }

        default:
            break;
    }

}

void hidCmdCallback(HID_CMD_Data_t* data) {
    char buf[] = "hidCmdCallback\r\n";
    HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
}

