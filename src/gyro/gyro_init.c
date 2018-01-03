#include "includes.h"
#include "spi_init.h"
#include "../gpio/board_init.h"


int gyro_init(void) 
{
    gpio_board_init();
    spi_init(SPI_BAUDRATEPRESCALER_32);
    gyro_configure();    
    spi_init(SPI_BAUDRATEPRESCALER_2);
    gyro_configure();
    gyro_exti_init();
    return(0);
}

void gyro_configure(void)
{
    //todo: do something.
}

void gyro_exti_init(void)
{
    //todo: do something.
}

