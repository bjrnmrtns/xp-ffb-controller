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

uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen);
void hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize);
void hidCmdCallback(HID_CMD_Data_t* data);


