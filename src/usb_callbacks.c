#include "stm32f3xx_hal.h"
#include "ffb_descriptor.h"
#include "tusb.h"

extern UART_HandleTypeDef huart1;

#define HID_BINTERVAL 0x01 // 1 = 1000hz, 2 = 500hz, 3 = 333hz 4 = 250hz, 5 = 200hz 6 = 166hz, 7 = 125hz...
#define USB_STRING_DESC_BUF_SIZE 32
#define USBD_VID     0x1209
#define USBD_PID     0xFFB0
const tusb_desc_device_t usb_devdesc_ffboard_composite =
        {
                .bLength            = sizeof(tusb_desc_device_t),
                .bDescriptorType    = TUSB_DESC_DEVICE,
                .bcdUSB             = 0x0200,

                // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
                .bDeviceClass       = TUSB_CLASS_MISC,
                .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
                .bDeviceProtocol    = MISC_PROTOCOL_IAD,
                .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

                .idVendor           = USBD_VID,
                .idProduct          = USBD_PID,
                .bcdDevice          = 0x0100,

                .iManufacturer      = 0x01,
                .iProduct           = 0x02,
                .iSerialNumber      = 0x03,

                .bNumConfigurations = 0x01
        };

const uint16_t usb_dev_desc_langId = 0x0409;
const uint8_t	usb_dev_desc_manufacturer[] = "Open_FFBoard";
const uint8_t usb_dev_desc_product[] = "FFBoard";
const uint8_t* usb_dev_desc_interfaces[] = { "FFBoard CDC", "FFBoard HID" };

FFB_BlockLoad_Feature_Data_t blockLoad_report =
        {
                .reportId = HID_ID_BLKLDREP,
        };

FFB_PIDPool_Feature_Data_t pool_report =
        {
                .reportId = HID_ID_POOLREP,
                .ramPoolSize = MAX_EFFECTS,
                .maxSimultaneousEffects = MAX_EFFECTS,
                .memoryManagement = 1,	// 0=DeviceManagedPool, 1=SharedParameterBlocks
        };

uint16_t _desc_str[USB_STRING_DESC_BUF_SIZE];

/*const uint8_t usb_cdc_conf[] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, 2, 0, (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN), TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(0, 4, 0x82, 8, 0x01, 0x81, 64),
};
*/
// Composite CDC and HID
const uint8_t usb_cdc_hid_conf[] =
        {
                // Config number, interface count, string index, total length, attribute, power in mA
                TUD_CONFIG_DESCRIPTOR(1, 3, 0, (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_INOUT_DESC_LEN), TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

                // 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size.
                TUD_CDC_DESCRIPTOR(0, 4, 0x82, 8, 0x01, 0x81, 64),

                // HID Descriptor. EP 83 and 2
                TUD_HID_INOUT_DESCRIPTOR(2, 5, HID_ITF_PROTOCOL_NONE, USB_HID_FFB_REPORT_DESC_SIZE, 0x83, 0x02, 64, HID_BINTERVAL),
        };


uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf){
    char buffer[] = "tud_hid_descriptor_report_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
    return hid_ffb_desc;
}

void tud_cdc_rx_cb(uint8_t itf){
    char buffer[] = "tud_cdc_rx_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}

void tud_cdc_tx_complete_cb(uint8_t itf){
    char buffer[] = "tud_cdc_tx_complete_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}

uint16_t hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
    char buf[] = "hidGet\r\n";
    HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
    uint8_t id = report_id - FFB_ID_OFFSET;

    switch(id){
        case HID_ID_BLKLDREP:
            //printf("Get Block Report\n");
            memcpy(buffer,&blockLoad_report,sizeof(FFB_BlockLoad_Feature_Data_t));
            return sizeof(FFB_BlockLoad_Feature_Data_t);
            break;
        case HID_ID_POOLREP:
            //printf("Get Pool Report\n");
            memcpy(buffer,&pool_report,sizeof(FFB_PIDPool_Feature_Data_t));
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
}

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


void hidCmdCallback(HID_CMD_Data_t* data) {
    char buf[] = "hidCmdCallback\r\n";
    HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){
    char buf[] = "tud_hid_set_report_cb\r\n";
    HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
    if(report_type == HID_REPORT_TYPE_INVALID && report_id == 0){
        report_id = *buffer;
    }

    hidOut(report_id,report_type,buffer,bufsize);

    if(report_id == HID_ID_HIDCMD) {
        hidCmdCallback((HID_CMD_Data_t*)(buffer));
    }
}

uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
    char buf[] = "tud_hid_get_report_cb\r\n";
    HAL_UART_Transmit(&huart1, &buf[0], strlen(buf), 10);
    return hidGet(report_id, report_type, buffer,reqlen);
}

void tud_hid_report_complete_cb(uint8_t itf, uint8_t const* report, uint8_t len){
    char buffer[] = "tud_hid_report_complete_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}

uint8_t const * tud_descriptor_device_cb(void)
{
    char buffer[] = "tud_descriptor_device\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
    return (uint8_t const *)&usb_devdesc_ffboard_composite;
}
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    char buffer[] = "tud_descriptor_configuration_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
    return usb_cdc_hid_conf;
}

void ascii_to_utf16(uint16_t* dest, const uint8_t* src) {
    int chr_count = strlen(src);
    if ( chr_count > 31 ) chr_count = 31;

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
        dest[i+1] = src[i];
    }
    dest[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    char buffer[] = "tud_descriptor_string_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
    uint16_t chr_count = 0;
    if (index == 0) // Language
    {
        _desc_str[1] = usb_dev_desc_langId;
        chr_count = 1;
    }else{
        if(index == usb_devdesc_ffboard_composite.iSerialNumber){
            const uint8_t buf[4];
            const int size = snprintf(&buf, 4, "%d%d%d", HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
            ascii_to_utf16(_desc_str, buf);
        }else if(index == usb_devdesc_ffboard_composite.iProduct){
            ascii_to_utf16(_desc_str, usb_dev_desc_product);
        }else if(index == usb_devdesc_ffboard_composite.iManufacturer){
            ascii_to_utf16(_desc_str, usb_dev_desc_manufacturer);
        }else if(index > 3 && (index - 4 < 3)){
            ascii_to_utf16(_desc_str, usb_dev_desc_interfaces[index - 4]);
        }else{
            return NULL;
        }
    }
    return _desc_str;
}
void tud_mount_cb(void)
{
    // start all stuff because usb is started
    char buffer[] = "tud_mount_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);

}
void tud_umount_cb(void)
{
    // stop all stuff because usb is essentially stopped
    char buffer[] = "tud_umount_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}
void tud_suspend_cb(bool remote_wakeup_en)
{
    // stop all stuff because usb is essentially stopped
    char buffer[] = "tud_suspend_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);

}
void tud_resume_cb(void)
{
    // start all stuff because usb is started
    char buffer[] = "tud_resume_cb\r\n";
    HAL_UART_Transmit(&huart1, &buffer[0], strlen(buffer), 10);
}
