#include "includes.h"
#include "flash.h"

FLASH_EraseInitTypeDef eraseInitStruct;

void erase_flash(uint32_t beginAddress, uint32_t endAddress)
{
    uint32_t pageError = 0;
    HAL_FLASH_Unlock();

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = beginAddress;
    eraseInitStruct.NbPages = (endAddress - beginAddress) / FLASH_PAGE_SIZE;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
    {
        while (1)
        {
            //TODO add error handler
        }
    }
}

void erase_page(uint32_t pageAddress)
{
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;
    SET_BIT(FLASH->CR, FLASH_CR_PER);
    WRITE_REG(FLASH->AR, pageAddress);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);
}