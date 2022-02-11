#pragma once

#include "ffb_descriptor.h"

#include "tusb.h"

typedef enum HidCmdType  {write = 0, request = 1, info = 2, writeAddr = 3, requestAddr = 4,ACK = 10, notFound = 13, notification = 14, err = 15} HidCmdType_t;

typedef struct
{
    uint8_t		reportId; // Report ID = 0xA1
    HidCmdType_t type;				// 0x01. Type of report. 0 = write, 1 = request
    uint16_t	clsid;				// 0x02 Class ID identifies the target class type
    uint8_t 	instance;			// 0x03 Class instance number to target a specific instance (often 0). 0xff for broadcast to all instances
    uint32_t	cmd;				// 0x04 Use this as an identifier for the command. upper 16 bits for class type
    uint64_t	data;				// 0x05 Use this to transfer data or the primary value
    uint64_t	addr;				// 0x06 Use this to transfer an optional address or second value (CAN for example)

} __attribute__((packed)) HID_CMD_Data_t;

// Internal struct for storing effects
typedef struct
{
    volatile uint8_t state;
    uint8_t type; // Type
    int16_t offset;				// Center point
    uint8_t gain;				// Scaler. often unused
    int16_t magnitude;			// High res intensity of effect
    int16_t startLevel;			// Ramp effect
    int16_t endLevel;			// Ramp effect
    uint8_t enableAxis;			// Active axis
    uint16_t directionX;		// angle (0=0 .. 36000=360deg)
    uint16_t directionY;		// angle (0=0 .. 36000=360deg)
#if MAX_AXIS == 3
    uint8_t directionZ; // angle (0=0 .. 255=360deg)
#endif
    uint8_t conditionsCount;
    FFB_Effect_Condition conditions[MAX_AXIS];
    int16_t phase;
    uint16_t period;
    uint32_t duration;					 // Duration in ms
    uint16_t attackLevel, fadeLevel; // Envelope effect
    uint32_t attackTime, fadeTime;	 // Envelope effect

    Biquad* filter[MAX_AXIS];  // Optional filter
    uint16_t startDelay;
    uint32_t startTime;	  // Elapsed time in ms before effect starts
    uint16_t samplePeriod;
    int useEnvelope;
} FFB_Effect;
void FFB_Effect_init(FFB_Effect* self);

void ffb_effects_init();

uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen);
void hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);
void hidCmdCallback(HID_CMD_Data_t* data);


