#pragma once


#include "stm32f1xx_hal.h"
#define flashADDR   0x0801F800


HAL_StatusTypeDef writeFlash(uint32_t address, uint32_t data);
uint32_t readFlash(uint32_t address);