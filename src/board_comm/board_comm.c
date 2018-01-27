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

volatile int saActive;
//SPI 3 is for the f4/f3
SPI_HandleTypeDef boardCommSPIHandle;
DMA_HandleTypeDef hdmaBoardCommSPIRx;
DMA_HandleTypeDef hdmaBoardCommSPITx;
uint8_t boardCommSpiRxBuffer[COM_BUFFER_SIZE];
uint8_t boardCommSpiTxBuffer[COM_BUFFER_SIZE];

volatile boardCommState_t boardCommState;

static void run_command(imufCommand_t* newCommand);

void board_comm_init(void) 
{
    saActive = 0;
    //default states for boardCommState
    boardCommState.commMode     = GTBCM_SETUP; 
    boardCommState.commMode     = GTBCM_GYRO_ACC_FILTER_F; //testing
    

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
    memset(boardCommSpiRxBuffer, 0, COM_BUFFER_SIZE);
    memset(boardCommSpiTxBuffer, 0, COM_BUFFER_SIZE);
    HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, COM_BUFFER_SIZE);
    //HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
}

void parse_imuf_command(imufCommand_t* newCommand, uint8_t* buffer){
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
            run_command(newCommand);
        }
    }
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

    if(saActive)
        return;

    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);

    parse_imuf_command(&newCommand, boardCommSpiRxBuffer);

    if(boardCommState.commMode == GTBCM_SETUP)
    {
        simpleDelay_ASM(3);
        //setup next command and notify F4 we're ready to talk
        //HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, GTBCM_SETUP);
        HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
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