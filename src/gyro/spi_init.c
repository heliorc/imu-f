#include "includes.h"
#include "../stm32/stm32.h"

SPI_HandleTypeDef gyroSpiHandle;
SPI_HandleTypeDef outSpiHandle;
IRQn_Type IRQn;
uint32_t IRQPriority;
uint32_t IRQSubPriority;

void spi_init(uint32_t baudRatePrescaler) 
{
    /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
    gyroSpiHandle.Instance               = GYRO_SPI;
    gyroSpiHandle.Init.Mode              = SPI_MODE_MASTER;
    gyroSpiHandle.Init.BaudRatePrescaler = baudRatePrescaler;
    gyroSpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    gyroSpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
    gyroSpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    gyroSpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    gyroSpiHandle.Init.CRCPolynomial     = 7;
    gyroSpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    gyroSpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    gyroSpiHandle.Init.NSS               = SPI_NSS_SOFT;
    gyroSpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    gyroSpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    gyroSpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
    
    HAL_SPI_DeInit(&gyroSpiHandle);

    if (HAL_SPI_Init(&gyroSpiHandle) != HAL_OK)
    {
        while(1);
    }
    if(!GYRO_CS_HARDWARE)
    {
        inline_digital_hi(GYRO_CS_PORT, GYRO_CS_PIN);
    }
    HAL_NVIC_SetPriority(IRQn, IRQPriority, IRQSubPriority);
    HAL_NVIC_EnableIRQ(IRQn);
}

//what am I reading from SPI?
void spi_de_init(void)
{
    HAL_SPI_DeInit(gyroSpiHandle);
    HAL_NVIC_DisableIRQ(IRQn);
}