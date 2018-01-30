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
volatile imufCommand_t imufCommandTx;
volatile imufCommand_t imufCommandRx;
volatile uint32_t timeBoardCommSetupIsr;

volatile boardCommState_t boardCommState;

static int run_command(volatile imufCommand_t* newCommand);
static void start_board_comm_isr(void);

void board_comm_init(void) 
{
    //default states for boardCommState
    boardCommState.commMode     = GTBCM_SETUP;     

    //set this pin to high since we're set at the default buffer size of GTBCM_SETUP
    HAL_GPIO_WritePin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);
    
    spiIrqCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_spi_irq_callback;
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, BOARD_COMM_SPI_ISR_PRE_PRI, BOARD_COMM_SPI_ISR_SUB_PRI);
    spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA, BOARD_COMM_SPI_RX_DMA_IRQn, BOARD_COMM_SPI_TX_DMA_IRQn);

    //put the SPI into IT mode, we check each byte as data comes in this way. The gyro transfers will use DMA mode
    start_board_comm_isr();
}


int parse_imuf_command(volatile imufCommand_t* newCommand){
    //copy received data into command structure

    if (!(newCommand->command && newCommand->command == newCommand->crc))
    {
        newCommand->command = BC_NONE;
        return 0;
    }
    else
    {
        return 1;
    }
}

static void start_board_comm_isr(void)
{
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0); //ready to communicate
    rewind_board_comm_spi();
    HAL_Delay(1);
    timeBoardCommSetupIsr = HAL_GetTick();
    memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
    memset((uint8_t *)&imufCommandRx, 0, sizeof(imufCommandRx));
    imufCommandTx.command = BC_IMUF_LISTENING; //we're  listening for a command
    imufCommandTx.crc     = BC_IMUF_LISTENING;
    HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, COM_BUFFER_SIZE);
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
}

void check_board_comm_setup_timeout(void)
{
    if(HAL_GetTick() - timeBoardCommSetupIsr > 10)
    {
        //timeout every 10 ms
        start_board_comm_isr();
    }
}

static int run_command(volatile imufCommand_t* newCommand)
{
    int validCommand = 0;
    HAL_StatusTypeDef check = HAL_TIMEOUT;
    switch (newCommand->command)
    {
        case BC_IMUF_RESTART:
            //repeat command to tell FC we have it, then restart
            memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
            imufCommandTx.command = BC_IMUF_RESTART;
            imufCommandTx.crc     = BC_IMUF_RESTART;
            HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
            check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, GTBCM_SETUP, 40);
            HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
            if (check == HAL_OK) //f4 got reply
            {
                BootToAddress(THIS_ADDRESS);
                return 0;
            }
            else
            {
                return 0;
            }
        break;
        case BC_IMUF_CALIBRATE:
            //might be good to make sure we're not flying when we run this command
            //this reply can only happen when not in setup mode
            if(boardCommState.commMode == GTBCM_SETUP)
            {
                memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
                imufCommandTx.command = BC_IMUF_CALIBRATE;
                imufCommandTx.crc     = BC_IMUF_CALIBRATE;
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
                check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, GTBCM_SETUP, 40);
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
                if (check == HAL_OK)
                {
                    return 0; //valid command, but return 0 does what we want to.
                }
                return 0;
            }
            calibratingGyro=1;
            return 1;
        break;
        case BC_IMUF_REPORT_INFO:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
                imufCommandTx.command = BC_IMUF_REPORT_INFO;
                imufCommandTx.crc     = BC_IMUF_REPORT_INFO;
                memcpy((uint8_t *)&imufCommandTx.param1, (uint8_t *)&flightVerson, sizeof(flightVerson));
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
                check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, GTBCM_SETUP, 40);
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
                if (check == HAL_OK)
                {
                    return 0; //valid command, but return 0 does what we want to.
                }
                return 0;
            }
            return 1;
        break;
        case BC_IMUF_SETUP:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                //let's pretend the f4 sent this for now
                newCommand->param1 = (GTBCM_GYRO_ONLY_FILTER_F << 24);
                //f4 needs to send all valid setup commands

                memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
                imufCommandTx.command = BC_IMUF_REPORT_INFO;
                imufCommandTx.crc     = BC_IMUF_REPORT_INFO;
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
                check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, GTBCM_SETUP, 40);
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
                if (check == HAL_OK)
                {
                    //message succsessfully sent to F4, switch to new comm mode now since f4 expects it now
                    boardCommState.commMode = (newCommand->param1 >> 24); //first 8 bits are comm mode, need to verify it's a valid command  though
                    //gyro.c will now handle communication
                    return 1;
                }
                return 0;
            }
            return 1;
        break;
        default:
            return 0;
        break;
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{

    //rx complete, do pin thing here
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);

    if (parse_imuf_command(&imufCommandRx))
    {
        if(imufCommandTx.command == BC_IMUF_LISTENING ) //we are expecting a command,
        {
            if (!run_command(&imufCommandRx))  //this command will handle  the message start handling
            {
                start_board_comm_isr();
            }
        }
        else
        {
            //we were sending data
            start_board_comm_isr();
        }
    }

}

void board_comm_spi_irq_callback(void)
{
    //irq handler
    if(boardCommState.commMode == GTBCM_SETUP)
    {

        HAL_SPI_IRQHandler(&boardCommSPIHandle);

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

    /* Re-enable SPI2 and DMA channels. */
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_RX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_TX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_SPI->CR1 |= SPI_CR1_SPE;
}