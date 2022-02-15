#pragma once
#include "stm32f3xx_hal.h"
extern UART_HandleTypeDef huart1;

#define LOG(msg) \
    HAL_UART_Transmit(&huart1, (uint8_t*)&msg, strlen(msg), 10);
