#pragma once


#include "stm32f1xx_hal.h"
#define flashADDR   0x0801F800


HAL_StatusTypeDef FLASH_ErasePage(uint32_t address);
HAL_StatusTypeDef FLASH_WriteWord(uint32_t address, uint32_t data);
uint32_t FLASH_ReadWord(uint32_t address);