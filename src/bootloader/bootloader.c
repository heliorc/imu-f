#include "includes.h"
#include "boothandler.h"
#include "bootloader_commands.h"
#include "hal_init.h"
#include "flash.h"

volatile BootloaderCommandTypdef bootloaderCommand;

static void bootloader_main(void)
{
    while(1)
    {
        switch (bootloaderCommand)
        {
            case BL_ERASE_ALL:
            erase_flash(APP_ADDRESS, FLASH_END);
            break;
            case BL_ERASE_PAGE:
            break;
            case BL_ERASE_ADDRESS:
            break;
            case BL_REPORT_INFO:
            break;
            case BL_BOOT_TO_APP:
            break;
            case BL_BOOT_TO_LOCATION:
            break;
            case BL_RESTART:
            break;
            case BL_WRITE_FIRMWARE:
            break;
            case BL_NONE:
            default:
            break;
        }
    }
}

void bootloader_start(void)
{
    bootloaderCommand = BL_NONE;
    hal_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN, 0); 
    HAL_Delay(500);
    if (HAL_GPIO_ReadPin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == (uint32_t)GPIO_PIN_RESET)
    {
        bootloader_main();
    }
    else 
    {
        BootToAddress(APP_ADDRESS);
    }
}
