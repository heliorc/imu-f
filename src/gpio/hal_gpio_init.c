#include "includes.h"

void hal_gpio_init_pin(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t pull, uint32_t alternate) 
{
    HAL_GPIO_DeInit(port,  pin);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin   = pin;
	GPIO_InitStruct.Mode  = mode;
	GPIO_InitStruct.Pull  = pull;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate  = alternate;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}