#include "includes.h"
#include "spi.h"

void board_comm_init(void) 
{
    spiIrqCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_spi_irq_callback;
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, 1, 3);
    spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA);

    if(!BOARD_COMM_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{
    //what we do after the transfer completes
}