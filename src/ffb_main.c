#include "ffb_effects.h"

#include "tusb.h"

int ffb_init() {
    ffb_effects_init();
    return tud_init(BOARD_DEVICE_RHPORT_NUM);
}

void ffb_main() {
    tud_task();
}