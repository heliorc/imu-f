#pragma once

#include "includes.h"

extern void single_gpio_init(GPIO_TypeDef * port, uint16_t pin, uint8_t af, GPIOMode_TypeDef mode, GPIOOType_TypeDef output, GPIOPuPd_TypeDef pull);
extern void board_gpio_init(void);