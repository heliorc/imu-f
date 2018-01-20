#include "includes.h"
#include "spi_init.h"
#include "gyro_start.h"


//SPI 2 is for the gyro
//SPI 2 is for the f4/f3
SPI_HandleTypeDef gyroSPIHandle;

DMA_HandleTypeDef hdmaGyroSPIRx;
DMA_HandleTypeDef hdmaGyroSPITx;


void gyro_configure(uint32_t baudRatePrescaler)
{
    spi_init(&gyroSPIHandle, GYRO_SPI, baudRatePrescaler, SPI_MODE_MASTER, GYRO_SPI_IRQn, 1, 2);
    spi_dma_init(&gyroSPIHandle, &hdmaGyroSPIRx, &hdmaGyroSPITx, GYRO_RX_DMA, GYRO_TX_DMA);
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