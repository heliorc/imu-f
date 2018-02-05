#include "includes.h"

void gpio_write_pin(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t pinState)
{
    if (pinState != 0)
    {
        GPIOx->BSRR = (uint32_t)GPIO_Pin;
    }
    else
    {
        GPIOx->BRR = (uint32_t)GPIO_Pin;
    }
}

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

void gpio_exti_init(GPIO_TypeDef * port, uint8_t portSrc, uint16_t pin, uint8_t pinSrc, uint32_t extiLine, uint32_t trigger, uint32_t extiIrqn, uint32_t prePri, uint32_t subPri)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    //set pin as input
    single_gpio_init(port, 0, pin, 0, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN);
    
    //tell system that you will use for interupt
    SYSCFG_EXTILineConfig(portSrc, pinSrc);
    
    EXTI_InitStruct.EXTI_Line = extiLine;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = trigger;

    //init exti
    EXTI_Init(&EXTI_InitStruct);
 
    //set vector
    NVIC_InitStruct.NVIC_IRQChannel = extiIrqn;
    //set priority 
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = prePri;
    //set sub priority
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = subPri;
    //enable interrupt
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    //add to NVIC
    NVIC_Init(&NVIC_InitStruct);
}