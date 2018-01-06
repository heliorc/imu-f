#include "includes.h"

void hal_init(GPIO_TypeDef* port, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t alternate) 
{
    HAL_GPIO_DeInit(port,  pin);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin   = pin;
	GPIO_InitStruct.Mode  = mode;
	GPIO_InitStruct.Pull  = pull;
	GPIO_InitStruct.Alternate  = alternate;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void gpio_board_init(void)
{
	uint32_t alt = 0;
	if(GYRO_CS_HARDWARE)
    {
        alt = GYRO_CS_ALTERNATE;
    }
	hal_init(GYRO_CS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, alt);
	hal_init(GYRO_SCK_PORT, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GYRO_SCK_ALTERNATE);
	hal_init(GYRO_MISO_PORT, GPIO_MODE_AF_PP, GPIO_NOPULL, GYRO_MISO_ALTERNATE);
	hal_init(GYRO_MOSI_PORT, GPIO_MODE_AF_PP, GPIO_NOPULL, GYRO_MOSI_ALTERNATE);
}