#include "includes.h"
#include "spi_init.h"


void gyro_init() 
{
    spiHandle = //what to set it to?
    spi_init(0);

    if (HAL_SPI_Init(&spiHandle) != HAL_OK)
    {
        while(1);
    }

    inline_digital_hi(ports[board.gyros[0].csPort], board.gyros[0].csPin);
}