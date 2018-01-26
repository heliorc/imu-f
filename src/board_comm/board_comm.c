#include "includes.h"
#include "spi.h"
#include "board_comm.h"
#include "includes.h"

#define simpleDelay_ASM(us) do {\
	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" (92*us) : "memory"\
		      );\
} while(0)


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

    //set this pin to high since we're set at the default buffer size of GTBCM_SETUP
    HAL_GPIO_WritePin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);
    
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
    memset(boardCommSpiTxBuffer, 0, COM_BUFFER_SIZE);
    snprintf((char *)boardCommSpiTxBuffer, 10, "Helios!");
    HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, GTBCM_SETUP+1);
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
}

void parse_imuf_command(imufCommand_t* newCommand, uint8_t* buffer){
    //copy received data into command structure
    memcpy(newCommand, buffer, sizeof(imufCommand_t));
    if (!(newCommand->command && newCommand->command == newCommand->crc))
    {
        newCommand->command = BC_NONE;
    }
}

static void run_command(imufCommand_t* newCommand)
{
    switch (newCommand->command)
    {
        case BC_IMUF_REPORT_INFO:
            memcpy(boardCommSpiTxBuffer, &flightVerson, sizeof(flightVerson));
            //callback handler will init the spi and pin for transmission
        break;
        case BC_IMUF_SETUP:
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

void syncHandler(void)
{
    if(boardCommSpiRxBuffer[0] != 'h')
    {
        volatile uint32_t huh1 = BOARD_COMM_SPI->CR1;
        volatile uint32_t huh2 = BOARD_COMM_SPI->CR2;
        volatile uint32_t huh3 = BOARD_COMM_SPI->SR;
        volatile uint32_t huh4 = BOARD_COMM_SPI->DR;

        volatile uint16_t cat1 = boardCommSPIHandle.TxXferSize;
        volatile uint16_t cat2 = boardCommSPIHandle.TxXferCount;

        //boardCommSPIHandle.TxXferCount.hdmatx.
        HAL_SPI_DeInit(&boardCommSPIHandle);
        simpleDelay_ASM(150);
        HAL_SPI_Init(&boardCommSPIHandle);
        HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, GTBCM_SETUP+1);
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{
    imufCommand_t newCommand;

    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);

    //syncHandler();
    //memset(boardCommSpiRxBuffer, 0, sizeof(boardCommSpiRxBuffer));
    parse_imuf_command(&newCommand, boardCommSpiRxBuffer);
    //snprintf((char *)boardCommSpiTxBuffer, 10, "Helios!");
    //run_command(&newCommand);

    if(boardCommState.commMode == GTBCM_SETUP)
    {
        simpleDelay_ASM(10);
        //setup next command and notify F4 we're ready to talk
        //HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, GTBCM_SETUP-1);
        HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
    }

}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* if(hspi->Instance == SPI1)
    {
		spiCallbackFunctionArray[0](hspi);
    }
    else  */
    if(hspi->Instance == BOARD_COMM_SPI)
    {
		volatile int poof = 1;
        if(boardCommSpiRxBuffer[0] != 'h')
        {
            for(int x=0;x<41;x++)
            {
                if(boardCommSpiRxBuffer[x] == 'h')
                {
                    uint8_t temp[41];
                    memcpy(temp+x, boardCommSpiTxBuffer, x-41);
                    memcpy(temp, boardCommSpiTxBuffer+x, 41-x);
                    memcpy(boardCommSpiTxBuffer, temp+x, 41);
                }
            }
        }
        
    }
    else if(hspi->Instance == GYRO_SPI)
    {
		volatile int pood = 1;
    }
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    /* if(hspi->Instance == SPI1)
    {
		spiCallbackFunctionArray[0](hspi);
    }
    else  */
    if(hspi->Instance == BOARD_COMM_SPI)
    {
		volatile int poo = 1;
    }
    else if(hspi->Instance == GYRO_SPI)
    {
		volatile int pee = 2;
    }
}