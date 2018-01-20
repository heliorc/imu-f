#include "includes.h"
#include "spi_init.h"


//SPI 2 is for the gyro
//SPI 2 is for the f4/f3
SPI_HandleTypeDef boardCommSPIHandle;

DMA_HandleTypeDef hdmaBoardCommSPIRx;
DMA_HandleTypeDef hdmaBoardCommSPITx;

void board_comm_init(void) 
{   
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, 1, 3);
    spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA);

    if(!BOARD_COMM_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}