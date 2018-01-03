#include "includes.h"
#include "gyro/passthrough_init.h"
#include "stm32/stm32_init.h"

#ifdef C3PUBL
int main(void)
{
    bootloader_start();
    return(0);
}
#else
int main(void)
{
    volatile int mouse = 0;
    stm32_init();
    gpio_board_init();
    gyro_passthrough_init();
    return(0);
}
#endif
