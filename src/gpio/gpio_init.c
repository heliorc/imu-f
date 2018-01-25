#include "includes.h"
#include "hal_gpio_init.h"

void gpio_init(void)
{

	//todo: "hal_gpio_init_pin" not a good name, maybe call it hal_gpio_init_wrapper

	if(GYRO_CS_TYPE)
	{
		hal_gpio_init_pin(GYRO_CS_PORT, GYRO_CS_PIN, GYRO_CS_MODE, GPIO_NOPULL, GYRO_CS_ALTERNATE);
		HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET); //pin needs to default to high
	}
	hal_gpio_init_pin(GYRO_SCK_PORT,  GYRO_SCK_PIN,  GPIO_MODE_AF_PP, GPIO_PULLUP, GYRO_SCK_ALTERNATE);
	hal_gpio_init_pin(GYRO_MISO_PORT, GYRO_MISO_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GYRO_MISO_ALTERNATE);
	hal_gpio_init_pin(GYRO_MOSI_PORT, GYRO_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GYRO_MOSI_ALTERNATE);

	if(BOARD_COMM_CS_TYPE)
	{
		hal_gpio_init_pin(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN, BOARD_COMM_CS_MODE, GPIO_NOPULL, BOARD_COMM_CS_ALTERNATE);
		HAL_GPIO_WritePin(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN, GPIO_PIN_SET); //pin needs to default to high
	}
	hal_gpio_init_pin(BOARD_COMM_SCK_PORT,  BOARD_COMM_SCK_PIN,  GPIO_MODE_AF_PP, GPIO_PULLUP, BOARD_COMM_SCK_ALTERNATE);
	hal_gpio_init_pin(BOARD_COMM_MISO_PORT, BOARD_COMM_MISO_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, BOARD_COMM_MISO_ALTERNATE);
	hal_gpio_init_pin(BOARD_COMM_MOSI_PORT, BOARD_COMM_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, BOARD_COMM_MOSI_ALTERNATE);

	//used to tell the F4 that the F3 is ready for communication via SPI
	hal_gpio_init_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);
	HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0); //pin needs to default to low

	//used to tell the F4 that the F3 is a set data size mode for communication via SPI
	hal_gpio_init_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0); 
	HAL_GPIO_WritePin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 0); //pin needs to default to low

}