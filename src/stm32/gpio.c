#include "includes.h"

uint32_t read_digital_input(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin)
{
    return (GPIOx->IDR & GPIO_Pin);
}

void single_gpio_init(GPIO_TypeDef * port, uint16_t pin_src, uint16_t pin, uint8_t af, GPIOMode_TypeDef mode, GPIOOType_TypeDef output, GPIOPuPd_TypeDef pull)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //map the pin
    if(af)
    {
        GPIO_PinAFConfig(port, pin_src, af);
    }

    //setup the gpio init struct
    GPIO_InitStructure.GPIO_Mode  = mode;
    GPIO_InitStructure.GPIO_OType = output;
    GPIO_InitStructure.GPIO_PuPd  = pull;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = pin;

    //init the gpio
    GPIO_Init(port, &GPIO_InitStructure);
}