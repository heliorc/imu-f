#include "includes.h"
#include "gpio/gpio_init.h"
#include "gyro/gyro_init.h"
#include "stm32/stm32_init.h"


int main(void) 
{
    volatile int mouse = 0;
    stm32_init();

    gpio_init(GPIOB, GPIO_PIN_5, 1);

    gyro_passthrough_init();

    return(0);
}