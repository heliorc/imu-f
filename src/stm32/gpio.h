#pragma once

#include "includes.h"

extern void gpio_write_pin(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t pinState);
extern uint32_t read_digital_input(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin);
extern void single_gpio_init(GPIO_TypeDef * port, uint16_t pin_src, uint16_t pin, uint8_t af, GPIOMode_TypeDef mode, GPIOOType_TypeDef output, GPIOPuPd_TypeDef pull);
extern void gpio_exti_init(GPIO_TypeDef * port, uint8_t portSrc, uint16_t pin, uint8_t pinSrc, uint32_t extiLine, uint32_t trigger, uint32_t extiIrqn, uint32_t prePri, uint32_t subPri);