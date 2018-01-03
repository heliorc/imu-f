#include "includes.h"
#include "gyro/passthrough_init.h"
#include "stm32/stm32_init.h"

int main(void) 
{
    volatile int mouse = 0;
    stm32_init();

    #ifdef C3PUBL
    bootloader_start();
    return(0);
    #endif

    gpio_board_init();

    gyro_passthrough_init();

    return(0);
}