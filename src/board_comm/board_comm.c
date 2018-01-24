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
    boardCommState.commMode     = GTBCM_SETUP; 
    boardCommState.commEnabled  = 0; 

    //set this pin to high since we're set at the default buffer size of DEFAULT_COM_SIZE
    HAL_GPIO_WritePin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_PIN_SET);

    spiIrqCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_spi_irq_callback;
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, BOARD_COMM_SPI_ISR_PRE_PRI, BOARD_COMM_SPI_ISR_SUB_PRI);
    spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA, BOARD_COMM_SPI_RX_DMA_IRQn, BOARD_COMM_SPI_TX_DMA_IRQn);

    //if master mode:
    //if(BOARD_COMM_CS_TYPE == NSS_SOFT)
    //{
    //    HAL_GPIO_WritePin(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN, GPIO_PIN_SET);
    //}

    //if slave mode:
    //comm works likes this, each commMode state has  a known size,
    //the Data Rdy pin goes high when the F3 is ready to communicate via SPI
    //the F3 has to know which mode it's in, it does this using 
    HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, DEFAULT_COM_SIZE);
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, GPIO_PIN_SET); //ready to communicate
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
            boardCommState.commMode   = newCommand->param1;
            boardCommState.bufferSize = newCommand->param2;

            //todo: validate params
            if(boardCommState.commMode != GTBCM_SETUP)
            {
                HAL_GPIO_WritePin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_PIN_RESET); //buffer size  is now set by ""
                boardCommState.commEnabled = 1; //if settings are valid we do this, else return an error
                //gyro will now handle communication
            }
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

    if(boardCommState.commMode == GTBCM_SETUP)
    {
        //setup next command and notify F4 we're ready to talk
        HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, DEFAULT_COM_SIZE);
        HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, GPIO_PIN_SET); //ready to communicate
    }
}
