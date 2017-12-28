#include "gpio/board_init.h"
#include "includes.h"


static void bootloader_main(void)
{
    //bootloader code
}

void bootloader_start(void);
{
    init_gpio_input(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);
    bootloader_wait();
    if (InlineIsPinStatusHi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == 1)
    {
        bootloader_main();
    }
    else 
    {
        BootToAddress(APP_ADDRESS);
    }
}

static void bootloader_wait(void)
{
    HAL_Delay(500);
}