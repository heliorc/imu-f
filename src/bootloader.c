#include "includes.h"


static void bootloader_wait(void);
static void bootloader_main(void);

static void bootloader_main(void)
{
    //bootloader code
    //needs to accept certain commands
    /*
        erase all
        erase page
        erase address
        report info
            report hardware version
            report firmware version
            report bootloader version
            report serial number
        boot to app
        boot to location
        restart
        write firmware
        report info
    */
}

void bootloader_start(void)
{
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