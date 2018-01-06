#include "includes.h"
#include "spi_init.h"

SPI_HandleTypeDef gyroHandle;

void gyro_configure(uint32_t baudRatePrescaler)
{
    spi_init(&gyroHandle, GYRO_SPI, baudRatePrescaler, SPI_MODE_MASTER, 0, 0);
}

void gyro_init(void) 
{   
    gyro_configure(SPI_BAUDRATEPRESCALER_32);
    gyro_configure(SPI_BAUDRATEPRESCALER_2);

    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}