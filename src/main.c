#include "includes.h"
#include "gpio/gpio_init.h"
#include "stm32/stm32_init.h"


int main(void) 
{
    volatile int mouse = 0;
    stm32_init();

    gpio_init(GPIOB, GPIO_PIN_5, 1);

    while(1)
    {
        GPIOB->ODR = 0xFFFF;
        //InlineDigitalHi(GPIOB, GPIO_PIN_5);
        //mouse++;
        GPIOB->ODR = 0x0000;
        //InlineDigitalLo(GPIOB, GPIO_PIN_5);
        //mouse++;
    }

    return(0);
}