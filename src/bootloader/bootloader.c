#include "includes.h"
#include "boothandler.h"
#include "bootloader_commands.h"
#include "hal_init.h"

BootloaderCommandTypdef bootloaderCommand;

static void bootloader_main(void)
{

}

void bootloader_start(void)
{
    bootloaderCommand = BL_NONE;
    hal_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN, 0); 
    HAL_Delay(500);
    if (InlineIsPinStatusHi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == 1)
    {
        bootloader_main();
    }
    else 
    {
        BootToAddress(APP_ADDRESS);
    }
}
