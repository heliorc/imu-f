#include "includes.h"
#include "gyro.h"
#include "spi.h"
#include "board_comm.h"
#include "includes.h"
#include "boothandler.h"

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
uint32_t boardCommSpiRxBufferPtr;
uint32_t boardCommSpiTxBufferPtr;

volatile boardCommState_t boardCommState;

static void run_command(imufCommand_t* newCommand);

void board_comm_init(void) 
{
    //default states for boardCommState
    boardCommState.commMode     = GTBCM_SETUP;     

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
    boardCommSpiRxBufferPtr = 0;
    boardCommSpiTxBufferPtr = 0;
    memset(boardCommSpiRxBuffer, 0, COM_BUFFER_SIZE);
    memset(boardCommSpiTxBuffer, 0, COM_BUFFER_SIZE);
    //put the SPI into IT mode, we check each byte as data comes in this way. The gyro transfers will use DMA mode
    HAL_SPI_TransmitReceive_IT(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, COM_BUFFER_SIZE);
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
}


int parse_imuf_command(imufCommand_t* newCommand, uint8_t* buffer){
    //copy received data into command structure
    if(buffer[0] == 'h')
    {
        if(buffer[0] == 'h' && buffer[1] == BC_IMUF_CALIBRATE && buffer[2] == BC_IMUF_CALIBRATE)
        {
            //runtime commands are different
            newCommand->command = BC_IMUF_CALIBRATE;
            newCommand->crc = BC_IMUF_CALIBRATE;
        }
        else
        {
            memcpy(newCommand, buffer+1, sizeof(imufCommand_t));
        }
        if (!(newCommand->command && newCommand->command == newCommand->crc))
        {
            newCommand->command = BC_NONE;
        }
        else
        {
            return 1;
        }
    }
    return 0;
}

static void run_command(imufCommand_t* newCommand)
{
    int validCommand = 0;
    switch (newCommand->command)
    {
        case BC_IMUF_RESTART:
            BootToAddress(THIS_ADDRESS);
        break;
        case BC_IMUF_CALIBRATE:
            //might be good to make sure we're not flying when we run this command
            calibratingGyro=1;
        break;
        case BC_IMUF_REPORT_INFO:
            boardCommSpiTxBuffer[0] = 'h';
            memcpy(boardCommSpiTxBuffer+1, &flightVerson, sizeof(flightVerson));
            //callback handler will init the spi and pin for transmission
        break;
        case BC_IMUF_SETUP:
            validCommand = 1;
            //todo: validate params
            if(validCommand)
            {
                //let's pretend the f4 sent this for now
                newCommand->param1 = GTBCM_GYRO_ONLY_FILTER_F; //f4 sends one extra byte so we can keep sync

                boardCommSpiTxBuffer[0] = 'h';
                boardCommSpiTxBuffer[1] = 'o';
                boardCommSpiTxBuffer[2] = 'k';
                HAL_StatusTypeDef check = HAL_TIMEOUT;
                check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, GTBCM_SETUP, 100);
                if (check == HAL_OK)
                {
                    //message succsessfully sent to F4, switch to new comm mode now
                    boardCommState.commMode   = newCommand->param1-1;
                    boardCommState.bufferSize = newCommand->param1-1;
                    HAL_GPIO_WritePin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_PIN_RESET); //buffer size  is now set by ""
                    //gyro will now handle communication
                }
            }
            else
            {
                boardCommSpiTxBuffer[0] = 'h';
                boardCommSpiTxBuffer[1] = 'n';
                boardCommSpiTxBuffer[2] = 'o';
            }
        break;
        default:
        break;
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{
    imufCommand_t newCommand;

    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);

    if (parse_imuf_command(&newCommand, boardCommSpiRxBuffer))
    {
        run_command(&newCommand);
    }

    if(boardCommState.commMode == GTBCM_SETUP)
    {
        simpleDelay_ASM(3);
        //setup next command and notify F4 we're ready to talk
        //HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, GTBCM_SETUP);
        HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
    }

}

//this runs in setup mode, it handler, all data gets parsed one byte at a time
static void spi_it_setup_mode_handler(void)
{
    imufCommand_t newCommand;

    //we're getting data in IT mode, let's turn off the data ready pin now that a data transaction is happening
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
    /* SPI in Receiver mode */
    if((__HAL_SPI_GET_IT_SOURCE(&boardCommSPIHandle, SPI_IT_RXNE) != RESET) && (__HAL_SPI_GET_FLAG(&boardCommSPIHandle, SPI_FLAG_RXNE) != RESET) && (__HAL_SPI_GET_FLAG(&boardCommSPIHandle, SPI_FLAG_OVR) == RESET))
    {
        /* Receive Transaction data */
        boardCommSpiRxBuffer[boardCommSpiRxBufferPtr++] = boardCommSPIHandle.Instance->DR;
        //check received data, send a 0 if 1 and vice-versa
        if(boardCommSpiRxBuffer[boardCommSpiRxBufferPtr-1] == 1)
        {
            boardCommSpiTxBuffer[boardCommSpiTxBufferPtr] = 0;
        }
        else
        {
            boardCommSpiTxBuffer[boardCommSpiTxBufferPtr] = 1;
        }
        //__HAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);
    }

      /* SPI in mode Transmitter ---------------------------------------------------*/
    if((__HAL_SPI_GET_IT_SOURCE(&boardCommSPIHandle, SPI_IT_TXE) != RESET) && (__HAL_SPI_GET_FLAG(&boardCommSPIHandle, SPI_FLAG_TXE) != RESET))
    {
        boardCommSPIHandle.Instance->DR = boardCommSpiTxBuffer[boardCommSpiRxBufferPtr++];
    }

    //transaction occured, parse rx buffer
    //if ( parse_imuf_command(&newCommand, boardCommSpiRxBufferPtr) )
    //{
    //    //there's a valid command, let's kill the SPI, resetup, and run the command
    //    rewind_board_comm_spi();
    //    HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, boardCommState.commMode); //profile: this takes 2.14us to run with O3 optimization
    //    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
    //    parse_imuf_command();
    //    run_command(&newCommand);
    //}
}

void board_comm_spi_irq_callback(void)
{
    //irq handler
    if(boardCommState.commMode == GTBCM_SETUP)
    {
        spi_it_setup_mode_handler();
    }
    else
    {
        HAL_SPI_IRQHandler(&boardCommSPIHandle);
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
		volatile int error_2 = 1;
    }
    else if(hspi->Instance == GYRO_SPI)
    {
		volatile int error_3 = 2;
    }
}

//To keep sync, we tell the SPI to expect more data than will be sent, when the proper data amount comes in we reset the SPI
void rewind_board_comm_spi(void)
{
    #define SPI_I2S_DMAReq_Tx               ((uint16_t)0x0002)
    #define SPI_I2S_DMAReq_Rx               ((uint16_t)0x0001)
    #define DMA_CCR1_EN                     ((uint16_t)0x0001)

    // Clear DMA1 global flags
    //SET_BIT(DMA1->IFCR, BOARD_COMM_RX_DMA_FLAG);
    //SET_BIT(DMA1->IFCR, BOARD_COMM_TX_DMA_FLAG);

    // Disable the DMA channels
    //BOARD_COMM_RX_DMA->CCR &= (uint16_t)(~DMA_CCR1_EN);
    //BOARD_COMM_TX_DMA->CCR &= (uint16_t)(~DMA_CCR1_EN);

    // Bring back SPI2 DMAs to start of Rx & Tx buffers -
    // CPAR/CMAR stay the same after disable, no need to
    // `restore` those.
   // BOARD_COMM_RX_DMA->CNDTR = COM_BUFFER_SIZE;
   // BOARD_COMM_TX_DMA->CNDTR = COM_BUFFER_SIZE * 2;

    /* Reset SPI2 (clears TXFIFO). */
    RCC->APB1RSTR |= BOARD_COMM_SPI_RST_MSK;
    RCC->APB1RSTR &= ~BOARD_COMM_SPI_RST_MSK;

    /* Reconfigure SPI2. */
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, BOARD_COMM_SPI_ISR_PRE_PRI, BOARD_COMM_SPI_ISR_SUB_PRI);
    //spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA, BOARD_COMM_SPI_RX_DMA_IRQn, BOARD_COMM_SPI_TX_DMA_IRQn);
    //HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, COM_BUFFER_SIZE);

    /* Re-enable SPI2 and DMA channels. */
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_RX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_TX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_SPI->CR1 |= SPI_CR1_SPE;
}