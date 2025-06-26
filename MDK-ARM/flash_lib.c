#include "stm32f1xx_hal.h"  // Adjust to your STM32 series

/**
  * @brief  Erases a single Flash page on STM32.
  * @note   Interrupts are disabled and Flash is unlocked internally.
  * @param  address  
  *         Any address within the target page.  
  *         Must be aligned to the start of a Flash page.  
  * @retval HAL_StatusTypeDef  
  *         - HAL_OK      : Page erased successfully.  
  *         - HAL_ERROR   : An error occurred during erase.  
  *         - HAL_BUSY    : Flash is busy.  
  *         - HAL_TIMEOUT : Operation timed out.
  */
HAL_StatusTypeDef FLASH_ErasePage(uint32_t address)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    __disable_irq();
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        __enable_irq();
        return status;
    }

    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = address;
    eraseInit.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);

    HAL_FLASH_Lock();
    __enable_irq();
    return status;
}

/**
  * @brief  Programs a 32-bit word into Flash memory.
  * @note   Interrupts are disabled and Flash is unlocked internally.
  * @param  address  
  *         4-byte aligned Flash address to program.  
  * @param  data     
  *         32-bit word to write into Flash.
  * @retval HAL_StatusTypeDef  
  *         - HAL_OK      : Word programmed successfully.  
  *         - HAL_ERROR   : An error occurred during programming.  
  *         - HAL_BUSY    : Flash is busy.  
  *         - HAL_TIMEOUT : Operation timed out.
  */
HAL_StatusTypeDef FLASH_WriteWord(uint32_t address, uint32_t data)
{
    HAL_StatusTypeDef status;

    __disable_irq();
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        __enable_irq();
        return status;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);

    HAL_FLASH_Lock();
    __enable_irq();
    return status;
}

/**
  * @brief  Reads a 32-bit word from Flash memory.
  * @param  address  
  *         4-byte aligned Flash address to read from.
  * @retval uint32_t 
  *         The 32-bit word retrieved from Flash.
  */
uint32_t FLASH_ReadWord(uint32_t address)
{
    return *(__IO uint32_t*)address;
}
