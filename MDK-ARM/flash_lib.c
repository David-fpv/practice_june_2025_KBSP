#include "flash_lib.h"
#include "stm32f1xx_hal.h"


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
uint8_t writeFlash (uint32_t addr, uint32_t value)
{
	HAL_StatusTypeDef status;
	FLASH_EraseInitTypeDef FlashErase; 
	uint32_t pageError = 0;
	
	__disable_irq();
	status = HAL_FLASH_Unlock();
	
	// Calculate the base address of the first flash page
	uint32_t startPageAddr = addr & ~(FLASH_PAGE_SIZE - 1);
	// Compute the address of the last byte of the 32-bit word (addr + 3)
	uint32_t lastAddr      = addr + sizeof(uint32_t) - 1;
	// Calculate the base address of the page containing that last byte
	uint32_t endPageAddr   = lastAddr & ~(FLASH_PAGE_SIZE - 1);

	// Set up the erase operation
	FlashErase.TypeErase   = FLASH_TYPEERASE_PAGES;
	FlashErase.PageAddress = startPageAddr;
	// If the first and last byte fall on different pages, erase two pages
	FlashErase.NbPages     = (endPageAddr != startPageAddr) ? 2 : 1;
	
	if (HAL_FLASHEx_Erase(&FlashErase, &pageError) != HAL_OK)
	{
		HAL_FLASH_Lock(); 
        __enable_irq();
		return HAL_ERROR;
	}

	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, value);
	
	__enable_irq();
	HAL_FLASH_Lock();  
	return status;
}


/**
  * @brief  Reads a 32-bit word from Flash memory.
  * @param  address  
  *         4-byte aligned Flash address to read from.
  * @retval uint32_t 
  *         The 32-bit word retrieved from Flash.
  */
uint32_t readFlash(uint32_t address)
{
    return *(__IO uint32_t*)address;
}
