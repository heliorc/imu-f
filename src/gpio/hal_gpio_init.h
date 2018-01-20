#include "includes.h"

extern void hal_gpio_init_pin(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t pull, uint32_t alternate);