#include "includes.h"
#include "gyro_init.h"
#include "shifty_kalman.h"

void gyro_passthrough_init() 
{
    gyro_init();
    while(1) 
    {
        //GET SHWIFTY/SHIFTY
        GPIOB->ODR = 0xFFFF;
        //InlineDigitalHi(GPIOB, GPIO_PIN_5);
        //mouse++;
        GPIOB->ODR = 0x0000;
        //InlineDigitalLo(GPIOB, GPIO_PIN_5);
        //mouse++;
    }
}