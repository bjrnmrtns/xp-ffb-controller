cmake_minimum_required(VERSION 3.20)

# Needed so cmake tries to compile test program as static, because test exe won't work for
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(stm32-tinyusb)

enable_language(C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
#set(CMAKE_C_EXTENSIONS OFF)

add_executable(stm32-tinyusb.elf
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_usb.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c

    Core/Src/stm32f3xx_it.c
    Core/Src/stm32f3xx_hal_msp.c
    Core/Src/syscalls.c
    Core/Src/system_stm32f3xx.c
    Core/Src/main.c
    Core/Src/sysmem.c

    Core/Startup/startup_stm32f303vctx.s

    src/ffb_descriptor.c
    src/ffb_effects.c
    src/usb_callbacks.c

    tinyusb/src/host/hub.c
    tinyusb/src/host/usbh.c
    tinyusb/src/host/usbh_control.c
    tinyusb/src/portable/nordic/nrf5x/dcd_nrf5x.c
    tinyusb/src/portable/synopsys/dwc2/dcd_dwc2.c
    tinyusb/src/portable/valentyusb/eptri/dcd_eptri.c
    tinyusb/src/portable/ehci/ehci.c
    tinyusb/src/portable/bridgetek/ft9xx/dcd_ft9xx.c
    tinyusb/src/portable/nxp/lpc_ip3511/dcd_lpc_ip3511.c
    tinyusb/src/portable/nxp/khci/dcd_khci.c
    tinyusb/src/portable/nxp/lpc17_40/dcd_lpc17_40.c
    tinyusb/src/portable/nxp/lpc17_40/hcd_lpc17_40.c
    tinyusb/src/portable/nxp/transdimension/hcd_transdimension.c
    tinyusb/src/portable/nxp/transdimension/dcd_transdimension.c
    tinyusb/src/portable/ohci/ohci.c
    tinyusb/src/portable/broadcom/synopsys/dcd_synopsys.c
    tinyusb/src/portable/template/dcd_template.c
    tinyusb/src/portable/mentor/musb/hcd_musb.c
    tinyusb/src/portable/mentor/musb/dcd_musb.c
    tinyusb/src/portable/st/synopsys/dcd_synopsys.c
    tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    tinyusb/src/portable/dialog/da146xx/dcd_da146xx.c
    tinyusb/src/portable/chipidea/ci_hs/dcd_ci_hs.c
    tinyusb/src/portable/chipidea/ci_hs/hcd_ci_hs.c
    tinyusb/src/portable/sunxi/dcd_sunxi_musb.c
    tinyusb/src/portable/microchip/samx7x/dcd_samx7x.c
    tinyusb/src/portable/microchip/samg/dcd_samg.c
    tinyusb/src/portable/microchip/pic32mz/dcd_pic32mz.c
    tinyusb/src/portable/microchip/samd/dcd_samd.c
    tinyusb/src/portable/nuvoton/nuc121/dcd_nuc121.c
    tinyusb/src/portable/nuvoton/nuc120/dcd_nuc120.c
    tinyusb/src/portable/nuvoton/nuc505/dcd_nuc505.c
    tinyusb/src/portable/renesas/usba/dcd_usba.c
    tinyusb/src/portable/renesas/usba/hcd_usba.c
    tinyusb/src/portable/espressif/esp32sx/dcd_esp32sx.c
    tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c
    tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c
    tinyusb/src/portable/raspberrypi/rp2040/hcd_rp2040.c
    tinyusb/src/portable/sony/cxd56/dcd_cxd56.c
    tinyusb/src/portable/mindmotion/mm32/dcd_mm32f327x_otg.c
    tinyusb/src/portable/ti/msp430x5xx/dcd_msp430x5xx.c
    tinyusb/src/common/tusb_fifo.c
    tinyusb/src/class/cdc/cdc_rndis_host.c
    tinyusb/src/class/cdc/cdc_host.c
    tinyusb/src/class/cdc/cdc_device.c
    tinyusb/src/class/video/video_device.c
    tinyusb/src/class/net/ncm_device.c
    tinyusb/src/class/net/ecm_rndis_device.c
    tinyusb/src/class/dfu/dfu_rt_device.c
    tinyusb/src/class/dfu/dfu_device.c
    tinyusb/src/class/midi/midi_device.c
    tinyusb/src/class/usbtmc/usbtmc_device.c
    tinyusb/src/class/bth/bth_device.c
    tinyusb/src/class/audio/audio_device.c
    tinyusb/src/class/msc/msc_host.c
    tinyusb/src/class/msc/msc_device.c
    tinyusb/src/class/hid/hid_device.c
    tinyusb/src/class/hid/hid_host.c
    tinyusb/src/class/vendor/vendor_host.c
    tinyusb/src/class/vendor/vendor_device.c
    tinyusb/src/device/usbd_control.c
    tinyusb/src/device/usbd.c
    tinyusb/src/tusb.c
)

target_compile_definitions(stm32-tinyusb.elf PRIVATE
    -DUSE_HAL_DRIVER
    -DSTM32F303xC
)

target_include_directories(stm32-tinyusb.elf PRIVATE
    Core/Inc
    Drivers/CMSIS/Device/ST/STM32F3xx/Include
    Drivers/CMSIS/Include
    Drivers/STM32F3xx_HAL_Driver/Inc
    Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
    tinyusb/src
)

target_compile_options(stm32-tinyusb.elf PRIVATE
    -mcpu=cortex-m4
    -specs=nosys.specs
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    #        -fdata-sections
    #        -ffunction-sections
    -specs=nano.specs
    -Wall
    $<$<CONFIG:Debug>:-Og>
)

target_link_options(stm32-tinyusb.elf PRIVATE
    -T${CMAKE_SOURCE_DIR}/STM32F303VCTX_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,-Map=stm32-tinyusb.map,--cref
    -Wl,--gc-sections -static
)

add_custom_command(TARGET stm32-tinyusb.elf
        POST_BUILD
        COMMAND arm-none-eabi-size stm32-tinyusb.elf)

add_custom_command(TARGET stm32-tinyusb.elf
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex stm32-tinyusb.elf stm32-tinyusb.hex
        COMMAND arm-none-eabi-objcopy -O binary stm32-tinyusb.elf stm32-tinyusb.bin)
