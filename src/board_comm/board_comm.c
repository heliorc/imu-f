#include "includes.h"
#include "spi.h"
#include "board_comm.h"

//SPI 3 is for the f4/f3
SPI_HandleTypeDef boardCommSPIHandle;
DMA_HandleTypeDef hdmaBoardCommSPIRx;
DMA_HandleTypeDef hdmaBoardCommSPITx;
uint8_t boardCommSpiRxBuffer[COM_BUFFER_SIZE];
uint8_t boardCommSpiTxBuffer[COM_BUFFER_SIZE];

volatile boardCommState_t boardCommState;

void board_comm_init(void) 
{
    //default states for boardCommState
    boardCommState.gyroPassMode = GTBCM_UNKNOWN; 
    boardCommState.commEnabled  = 0; 

    spiIrqCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_spi_irq_callback;
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, BOARD_COMM_SPI_ISR_PRE_PRI, BOARD_COMM_SPI_ISR_SUB_PRI);
    spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA, BOARD_COMM_SPI_RX_DMA_IRQn, BOARD_COMM_SPI_TX_DMA_IRQn);

    //if master mode:
    //if(BOARD_COMM_CS_TYPE == NSS_SOFT)
    //{
    //    HAL_GPIO_WritePin(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN, GPIO_PIN_SET);
    //}

    //if slave mode:
    while(!boardCommState.commEnabled || !boardCommState.gyroPassMode)
    {
        //check each byte, we have no idea how arge the buffers are going to need to be yet
        HAL_SPI_TransmitReceive(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, 1, 100);
        //
    }
}

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
            boardCommState.gyroPassMode = newCommand->param1;
            //if settings are valid we do this, else return an error
            boardCommState.commEnabled  = 1;
        break;
        default:
        break;
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{
    imufCommand_t newCommand;

    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, GPIO_PIN_RESET);

    parse_imuf_command(&newCommand, boardCommSpiRxBuffer);
    run_command(&newCommand);
    memset(boardCommSpiRxBuffer, 0, COM_BUFFER_SIZE);
    //setup for next DMA transfer
    //HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, COM_BUFFER_SIZE);
}
