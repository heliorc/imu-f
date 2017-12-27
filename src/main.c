#include "includes.h"
#include "gpio/gpio_init.h"
#include "gyro/passthrough_init.h"
#include "stm32/stm32_init.h"

int main(void) 
{
    volatile int mouse = 0;
    stm32_init();

    gyro_init();

    return(0);
}