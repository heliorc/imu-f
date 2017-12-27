#include "includes.h"

void gyro_passthrough_init(void) 
{
    while(1) 
    {
        GPIOB->ODR = 0xFFFF;
        //InlineDigitalHi(GPIOB, GPIO_PIN_5);
        //mouse++;
        GPIOB->ODR = 0x0000;
        //InlineDigitalLo(GPIOB, GPIO_PIN_5);
        //mouse++;
    }
}