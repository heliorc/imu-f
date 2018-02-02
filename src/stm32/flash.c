#include "includes.h"
#include "flash.h"

uint32_t numberOfPages, eraseCounter;
__IO FLASH_Status flashStatus;

int flashUnlocked = 0;

void erase_flash(uint32_t beginAddress, uint32_t endAddress)
{

    prepare_flash_for_program();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    numberOfPages = (endAddress - beginAddress) / FLASH_PAGE_SIZE;
    flashStatus = FLASH_COMPLETE;

    for(eraseCounter = 0; (eraseCounter < numberOfPages) && (flashStatus == FLASH_COMPLETE); eraseCounter++)
    {
        if (FLASH_ErasePage(beginAddress + (FLASH_PAGE_SIZE * eraseCounter))!= FLASH_COMPLETE)
        {
            while(1)
            {
                //error handler
            }
        }
    }

    end_flash_for_program();
}

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
    else
    {
        erase_flash(beginAddress, endAddress);
    }
}

void prepare_flash_for_program(void)
{
    FLASH_Unlock();
    flashUnlocked = 1;
}

void end_flash_for_program(void)
{
    flashUnlocked = 0;
    FLASH_Lock();
}

int flash_program_word(uint32_t address, uint32_t data)
{
    if (flashUnlocked == 0)
    {
        return 0;
    }
    if (FLASH_ProgramWord(address, data) == FLASH_COMPLETE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}