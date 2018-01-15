#include "includes.h"
#include "flash.h"

FLASH_EraseInitTypeDef eraseInitStruct;
int flashUnlocked = 0;

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
    HAL_FLASH_Lock();
}

// void erase_page(uint32_t pageAddress)
// {
//     HAL_FLASH_Unlock();
//     WRITE_REG(FLASH->AR, pageAddress);
//     SET_BIT(FLASH->CR, FLASH_CR_STRT);
// }

void erase_range(uint32_t beginAddress, uint32_t endAddress)
{
    if (beginAddress > endAddress)
    {
        while (1)
        {
            //TODO add error handler
        }
    }
    else if (beginAddress < APP_ADDRESS)
    {
        while (1)
        {
            //TODO add error handler
        }
    }
    else{
        erase_flash(beginAddress, endAddress);
    }
}

void prepare_flash_for_program(void)
{
    HAL_FLASH_Unlock();
    flashUnlocked = 1;
}

void end_flash_for_program(void)
{
    flashUnlocked = 0;
    HAL_FLASH_Lock();
}

int flash_program_word(uint32_t address, uint32_t data)
{
    if (flashUnlocked == 0)
    {
        return 0;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}