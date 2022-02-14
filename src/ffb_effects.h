#pragma once

#include "ffb_descriptor.h"
#include "ffb_effects_calc.h"

#include "tusb.h"


uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen);
void hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);
void hidCmdCallback(HID_CMD_Data_t* data);

typedef struct {
    uint32_t hid_out_period; // ms since last out report for measuring update rate
    uint8_t last_effect_id;
    uint16_t used_effects;
    int ffb_active;
    FFB_BlockLoad_Feature_Data_t blockLoad_report;
    FFB_PIDPool_Feature_Data_t pool_report;
    FFB_Effect effects[MAX_EFFECTS];
    EffectsCalculator effectsCalculator;
    reportFFB_status_t reportFFBStatus;
    uint32_t lastOut;
} hid_ffb_t;
void hid_ffb_t_init(hid_ffb_t* self);
void update(hid_ffb_t* hid_ffb);
