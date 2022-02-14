#include "ffb_main.h"
#include "ffb_effects.h"

#include "tusb.h"

extern hid_ffb_t hid_ffb;

int ffb_init() {
    hid_ffb_t_init(&hid_ffb);
    return tud_init(BOARD_DEVICE_RHPORT_NUM);
}

void ffb_main() {
    tud_task();
    update(&hid_ffb);
}