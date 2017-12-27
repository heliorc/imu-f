#include "includes.h"
#include "spi_init.h"
#include "../gpio/gpio_init.h"

static void spi_gpio_init(void);
static void spi_init(uint32_t baudRatePrescaler);

int gyro_init(void) 
{
    gyroSpiHandle = //what to set it to??
    outSpiHandle = //what to set it to??
    spi_gpio_init();
    spi_init(SPI_BAUDRATEPRESCALER_32);
    gyro_configure();
    spi_init(SPI_BAUDRATEPRESCALER_2);
    gyro_exti_init();
    return(1);
}

//maybe an unnecessary abstraction?
static void spi_gpio_init(void)
{
    gpio_init();
}

void gyro_configure(void)
{
    //todo: do something.
}

void gyro_exti_init(void)
{
    //todo: do something.
}

