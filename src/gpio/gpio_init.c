#include "includes.h"

void gpio_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t on)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_GPIO_DeInit(GYRO_CS_PORT,  GYRO_CS_PIN);
    HAL_GPIO_DeInit(GYRO_MISO_PORT,  GYRO_MISO_PIN);
    HAL_GPIO_DeInit(GYRO_MOSI_PORT,  GYRO_MOSI_PIN);
    HAL_GPIO_DeInit(GYRO_SCK_PORT,  GYRO_SCK_PIN);

	GPIO_InitStruct.Pin   = GYRO_CS_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    if(GYRO_CS_HARDWARE)
    {
        GPIO_InitStruct.Alternate = GYRO_CS_ALTERNATE;
    }
	HAL_GPIO_Init(GYRO_CS_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin   = GYRO_SCK_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GYRO_SCK_ALTERNATE;
	HAL_GPIO_Init(GYRO_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GYRO_MISO_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GYRO_MISO_ALTERNATE;
	HAL_GPIO_Init(GYRO_MISO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GYRO_MOSI_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GYRO_MOSI_ALTERNATE;
	HAL_GPIO_Init(GYRO_MOSI_PORT, &GPIO_InitStruct);

    if(!GYRO_CS_HARDWARE)
    {
        inline_digital_hi(GYRO_CS_PORT, GYRO_CS_PIN);
    }

}