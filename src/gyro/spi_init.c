#include "includes.h"

void init_handle(SPI_HandleTypeDef* spiHandle, IRQn_Type irq)
{
    HAL_SPI_DeInit(spiHandle);
    HAL_NVIC_DisableIRQ(irq);

    if (HAL_SPI_Init(spiHandle) != HAL_OK)
    {
        //TODO: handle this error.
        while(1);
    }
}

void spi_init(SPI_HandleTypeDef* spiHandle, SPI_TypeDef* instance, uint32_t baudscaler, uint32_t spi_mode, uint32_t irqp, uint32_t irqsp)
{
    /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
    spiHandle->Instance               = instance;
    spiHandle->Init.Mode              = spi_mode;
    spiHandle->Init.BaudRatePrescaler = baudscaler;
    spiHandle->Init.Direction         = SPI_DIRECTION_2LINES;
    spiHandle->Init.CLKPhase          = SPI_PHASE_2EDGE;
    spiHandle->Init.CLKPolarity       = SPI_POLARITY_HIGH;
    spiHandle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    spiHandle->Init.CRCPolynomial     = 7;
    spiHandle->Init.DataSize          = SPI_DATASIZE_8BIT;
    spiHandle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    spiHandle->Init.NSS               = SPI_NSS_SOFT;
    spiHandle->Init.TIMode            = SPI_TIMODE_DISABLE;
    spiHandle->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    spiHandle->Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
    init_handle((spiHandle), GYRO_SPI_IRQn);

    HAL_NVIC_SetPriority(SPI1_IRQn, irqp, irqsp);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}
