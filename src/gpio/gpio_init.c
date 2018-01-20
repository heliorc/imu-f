#include "includes.h"
#include "hal_gpio_init.h"

void gpio_init(void)
{
	uint32_t alt = 0;
	if(GYRO_CS_HARDWARE)
    {
        alt = GYRO_CS_ALTERNATE;
	}
	//todo: not a good name, maybe call it hal_gpio_init_wrapper
	hal_gpio_init_pin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, alt);
	hal_gpio_init_pin(GYRO_SCK_PORT, GYRO_SCK_PIN, GPIO_MODE_AF_PP, GPIO_PULLDOWN, GYRO_SCK_ALTERNATE);
	hal_gpio_init_pin(GYRO_MISO_PORT, GYRO_MISO_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GYRO_MISO_ALTERNATE);
	hal_gpio_init_pin(GYRO_MOSI_PORT, GYRO_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GYRO_MOSI_ALTERNATE);
}