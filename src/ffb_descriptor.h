#ifndef SRC_FFB_DESCRIPTOR_H_
#define SRC_FFB_DESCRIPTOR_H_

#include <stdint.h>

#define USB_HID_FFB_REPORT_DESC_SIZE 1229//1378

extern const uint8_t hid_ffb_desc[USB_HID_FFB_REPORT_DESC_SIZE];

#define FFB_ID_OFFSET 0x00
#define MAX_EFFECTS 40
#define MAX_AXIS 2

// HID Descriptor definitions - Axes
#define HID_USAGE_X		0x30
#define HID_USAGE_Y		0x31
#define HID_USAGE_Z		0x32
#define HID_USAGE_RX	0x33
#define HID_USAGE_RY	0x34
#define HID_USAGE_RZ	0x35
#define HID_USAGE_SL0	0x36
#define HID_USAGE_SL1	0x37
#define HID_USAGE_WHL	0x38
#define HID_USAGE_POV	0x39

// HID Descriptor definitions - FFB Effects
#define HID_USAGE_CONST 0x26    //    Usage ET Constant Force
#define HID_USAGE_RAMP  0x27    //    Usage ET Ramp
#define HID_USAGE_SQUR  0x30    //    Usage ET Square
#define HID_USAGE_SINE  0x31    //    Usage ET Sine
#define HID_USAGE_TRNG  0x32    //    Usage ET Triangle
#define HID_USAGE_STUP  0x33    //    Usage ET Sawtooth Up
#define HID_USAGE_STDN  0x34    //    Usage ET Sawtooth Down
#define HID_USAGE_SPRNG 0x40    //    Usage ET Spring
#define HID_USAGE_DMPR  0x41    //    Usage ET Damper
#define HID_USAGE_INRT  0x42    //    Usage ET Inertia
#define HID_USAGE_FRIC  0x43    //    Usage ET Friction


// HID Descriptor definitions - FFB Report IDs
#define HID_ID_STATE	0x02	// Usage PID State report

#define HID_ID_EFFREP	0x01	// Usage Set Effect Report
#define HID_ID_ENVREP	0x02	// Usage Set Envelope Report
#define HID_ID_CONDREP	0x03	// Usage Set Condition Report
#define HID_ID_PRIDREP	0x04	// Usage Set Periodic Report
#define HID_ID_CONSTREP	0x05	// Usage Set Constant Force Report
#define HID_ID_RAMPREP	0x06	// Usage Set Ramp Force Report
#define HID_ID_CSTMREP	0x07	// Usage Custom Force Data Report
#define HID_ID_SMPLREP	0x08	// Usage Download Force Sample
#define HID_ID_EFOPREP	0x0A	// Usage Effect Operation Report
#define HID_ID_BLKFRREP	0x0B	// Usage PID Block Free Report
#define HID_ID_CTRLREP	0x0C	// Usage PID Device Control
#define HID_ID_GAINREP	0x0D	// Usage Device Gain Report
#define HID_ID_SETCREP	0x0E	// Usage Set Custom Force Report
// Features
#define HID_ID_NEWEFREP	0x11	// Usage Create New Effect Report
#define HID_ID_BLKLDREP	0x12	// Usage Block Load Report
#define HID_ID_POOLREP	0x13	// Usage PID Pool Report

// Control
#define HID_ID_CUSTOMCMD 0xAF   // Custom cmd (old. reserved)
#define HID_ID_HIDCMD	 0xA1   // HID cmd
#define HID_ID_STRCMD	 0xAC   // HID cmd as string. reserved
//#define HID_ID_CUSTOMCMD_IN 0xA2   // Custom cmd in. reserved
//#define HID_ID_CUSTOMCMD_OUT 0xA1   // Custom cmd out. reserved


#define FFB_EFFECT_NONE			0x00
#define FFB_EFFECT_CONSTANT		0x01
#define FFB_EFFECT_RAMP			0x02
#define FFB_EFFECT_SQUARE 		0x03
#define FFB_EFFECT_SINE 		0x04
#define FFB_EFFECT_TRIANGLE		0x05
#define FFB_EFFECT_SAWTOOTHUP	0x06
#define FFB_EFFECT_SAWTOOTHDOWN	0x07
#define FFB_EFFECT_SPRING		0x08
#define FFB_EFFECT_DAMPER		0x09
#define FFB_EFFECT_INERTIA		0x0A
#define FFB_EFFECT_FRICTION		0x0B
#define FFB_EFFECT_CUSTOM		0x0C

#define HID_ACTUATOR_POWER 		0x08
#define HID_SAFETY_SWITCH 		0x04
#define HID_ENABLE_ACTUATORS 	0x02
#define HID_EFFECT_PAUSE		0x01
#define HID_ENABLE_ACTUATORS_MASK 0xFD
#define HID_EFFECT_PLAYING 		0x10

#define HID_DIRECTION_ENABLE 0x04
#define FFB_EFFECT_DURATION_INFINITE 0xffff


typedef struct
{
	uint8_t	reportId;// = HID_ID_BLKLDREP;
	uint8_t effectBlockIndex;	// 1..max_effects
	uint8_t	loadStatus;	// 1=Success,2=Full,3=Error
	uint16_t	ramPoolAvailable;
} __attribute__((packed)) FFB_BlockLoad_Feature_Data_t;

typedef struct
{
	uint8_t	reportId;// = HID_ID_POOLREP;
	uint16_t	ramPoolSize;// = MAX_EFFECTS;
	uint8_t		maxSimultaneousEffects;// = MAX_EFFECTS;
	uint8_t		memoryManagement// = 1;	// 0=DeviceManagedPool, 1=SharedParameterBlocks
} __attribute__((packed)) FFB_PIDPool_Feature_Data_t;

typedef struct
{
    int16_t cpOffset;
    int16_t positiveCoefficient;
    int16_t negativeCoefficient;
    uint16_t positiveSaturation;
    uint16_t negativeSaturation;
    uint16_t deadBand;

} __attribute__((packed)) FFB_Effect_Condition;

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

    //std::unique_ptr<Biquad> filter[MAX_AXIS];  // Optional filter
    uint16_t startDelay;
    uint32_t startTime;	  // Elapsed time in ms before effect starts
    uint16_t samplePeriod;
    int useEnvelope;
} FFB_Effect;

typedef struct
{
    uint8_t		reportId;
    uint8_t	effectType;	// Effect type ID
    uint16_t	byteCount;	// Size of custom effects
} __attribute__((packed)) FFB_CreateNewEffect_Feature_Data_t;

#endif /* SRC_FFB_DESCRIPTOR_H_ */
