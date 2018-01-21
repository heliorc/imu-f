#include "includes.h"
#include "spi.h"
#include "board_comm.h"

boardCommand_t boardCommand;

static void run_board_command(boardCommand_t* boardCommand);

static void run_board_command(boardCommand_t* boardCommand)
{
    switch (boardCommand->command)
    {
        case BC_GYRO_SETUP:
        break;
        case BC_NONE:
        default:
        break;
    }
}

void board_comm_init(void) 
{
    spiIrqCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_spi_irq_callback;
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, 1, 3);
    spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA);

    if(!BOARD_COMM_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN, GPIO_PIN_SET);
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{
    //what we do after the transfer completes
    memcpy(&boardCommand, boardCommSpiRxBuffer, sizeof(boardCommand_t));
    if (boardCommand.command && boardCommand.command == boardCommand.crc){
        run_board_command(&boardCommand);
    }
    memset(boardCommSpiRxBuffer, 0, SPI_BUFFER_SIZE);
    //setup for next DMA transfer
    HAL_SPI_TransmitReceive_IT(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, SPI_BUFFER_SIZE);
}