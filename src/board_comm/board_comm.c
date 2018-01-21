#include "includes.h"
#include "spi.h"
#include "board_comm.h"

//SPI 3 is for the f4/f3
SPI_HandleTypeDef boardCommSPIHandle;
DMA_HandleTypeDef hdmaBoardCommSPIRx;
DMA_HandleTypeDef hdmaBoardCommSPITx;
uint8_t boardCommSpiRxBuffer[COM_BUFFER_SIZE];
uint8_t boardCommSpiTxBuffer[COM_BUFFER_SIZE];

void parse_imuf_command(imufCommand_t* newCommand, uint8_t* buffer){
    //copy received data into command structure
    memcpy(newCommand, boardCommSpiRxBuffer, sizeof(imufCommand_t));
    if (!(newCommand->command && newCommand->command == newCommand->crc)){
        newCommand->command = NONE;
    }
}

static void run_command(imufCommand_t* newCommand)
{
    switch (newCommand->command)
    {
        case BC_GYRO_SETUP:
        break;
        default:
        break;
    }
}

void board_comm_init(void) 
{
    spiIrqCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_spi_irq_callback;
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, BOARD_COMM_SPI_ISR_PRE_PRI, BOARD_COMM_SPI_ISR_SUB_PRI);
    spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA);

    if(!BOARD_COMM_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN, GPIO_PIN_SET);
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{
    imufCommand_t newCommand;
    parse_imuf_command(&newCommand, boardCommSpiRxBuffer);
    run_command(&newCommand);
    memset(boardCommSpiRxBuffer, 0, COM_BUFFER_SIZE);
    //setup for next DMA transfer
    HAL_SPI_TransmitReceive_IT(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, COM_BUFFER_SIZE);
}
