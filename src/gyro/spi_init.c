#include "includes.h"
#include "../stm32/stm32.h"

SPI_HandleTypeDef spiHandle;

void spi_init(uint32_t baudRatePrescaler) 
{
    /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
    spiHandle.Instance               = GYRO_SPI;
    spiHandle.Init.Mode              = SPI_MODE_MASTER;
    spiHandle.Init.BaudRatePrescaler = baudRatePrescaler;
    spiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    spiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
    spiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    spiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    spiHandle.Init.CRCPolynomial     = 7;
    spiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    spiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    spiHandle.Init.NSS               = SPI_NSS_SOFT;
    spiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    spiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    spiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
}