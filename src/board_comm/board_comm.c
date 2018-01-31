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

static void run_command(volatile imufCommand_t* newCommand);
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

    //put the SPI into DMA
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
    SPI_HandleTypeDef* hspi = &boardCommSPIHandle;
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0); //ready to communicate
    rewind_board_comm_spi();
    simpleDelay_ASM(5);

    timeBoardCommSetupIsr = HAL_GetTick();
    memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
    memset((uint8_t *)&imufCommandRx, 0, sizeof(imufCommandRx));
    imufCommandTx.command = BC_IMUF_LISTENING; //we're  listening for a command
    imufCommandTx.crc     = BC_IMUF_LISTENING;
    volatile uint32_t cat = sizeof(imufCommandTx);

    HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, COM_BUFFER_SIZE+2);
    //hspi->pTxBuffPtr  = (uint8_t*)&imufCommandTx;
    //hspi->TxXferSize  = COM_BUFFER_SIZE;
    //hspi->TxXferCount = COM_BUFFER_SIZE;
    //hspi->pRxBuffPtr  = (uint8_t*)&imufCommandRx;
    //hspi->RxXferSize  = COM_BUFFER_SIZE;
    //hspi->RxXferCount = COM_BUFFER_SIZE;
    //SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN);
    //SET_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN);
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //ready to communicate
}

void check_board_comm_setup_timeout(void)
{

    if(HAL_GetTick() - timeBoardCommSetupIsr > 2000)
    {
        //timeout every 10 ms
        start_board_comm_isr();
    }
}

static void run_command(volatile imufCommand_t* newCommand)
{
    int validCommand = 0;
    HAL_StatusTypeDef check = HAL_TIMEOUT;
    switch (newCommand->command)
    {
        case BC_IMUF_CALIBRATE:
            //might be good to make sure we're not flying when we run this command
            //this reply can only happen when not in setup mode
            if(boardCommState.commMode == GTBCM_SETUP)
            {
                memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
                imufCommandTx.command = BC_IMUF_CALIBRATE;
                imufCommandTx.crc     = BC_IMUF_CALIBRATE;
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
                check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, GTBCM_SETUP+2, 40);
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
            }
            calibratingGyro=1;
        break;
        case BC_IMUF_REPORT_INFO:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
                memcpy((uint8_t *)&imufCommandTx.param1, (uint8_t *)&flightVerson, sizeof(flightVerson));
                imufCommandTx.command = BC_IMUF_REPORT_INFO;
                imufCommandTx.crc     = BC_IMUF_REPORT_INFO;
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
                check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, GTBCM_SETUP+2, 40);
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
            }
        break;
        case BC_IMUF_SETUP:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                //let's pretend the f4 sent this for now
                newCommand->param1 = (GTBCM_GYRO_ACC_FILTER_F << 24);
                //f4 needs to send all valid setup commands

                memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
                imufCommandTx.command = BC_IMUF_SETUP;
                imufCommandTx.crc     = BC_IMUF_SETUP;
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
                check = HAL_SPI_TransmitReceive(&boardCommSPIHandle, (uint8_t *)&imufCommandTx, (uint8_t *)&imufCommandRx, GTBCM_SETUP+2, 40);
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
                if (check == HAL_OK)
                {
                    //message succsessfully sent to F4, switch to new comm mode now since f4 expects it now
                    boardCommState.commMode = (GTBCM_GYRO_ACC_FILTER_F); //first 8 bits are comm mode, need to verify it's a valid command  though
                    //gyro.c will now handle communication
                }
            }
        break;
        default:
        break;
    }
}

void board_comm_callback_function(SPI_HandleTypeDef *hspi)
{

    //rx complete, do pin thing here
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
    timeBoardCommSetupIsr = HAL_GetTick();

    if ( imufCommandRx.command == 0x7f7f7f7F)
    {
        BootToAddress(THIS_ADDRESS);
    }
    if (parse_imuf_command(&imufCommandRx))
    {
        run_command(&imufCommandRx);  //this command will handle the message start handling
    }

    //restart listening if in setup mode
    if(boardCommState.commMode == GTBCM_SETUP)
        timeBoardCommSetupIsr = 0;

}

void board_comm_spi_irq_callback(void)
{
    //irq handler
    HAL_SPI_IRQHandler(&boardCommSPIHandle);
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
    //BOARD_COMM_RX_DMA->CNDTR = GTBCM_SETUP;
    //BOARD_COMM_TX_DMA->CNDTR = GTBCM_SETUP;

    /* Reset SPI2 (clears TXFIFO). */
    RCC->APB1RSTR |= BOARD_COMM_SPI_RST_MSK;
    RCC->APB1RSTR &= ~BOARD_COMM_SPI_RST_MSK;

    /* Reconfigure SPI2. */
    //spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, BOARD_COMM_SPI_ISR_PRE_PRI, BOARD_COMM_SPI_ISR_SUB_PRI);
    //spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA, BOARD_COMM_SPI_RX_DMA_IRQn, BOARD_COMM_SPI_TX_DMA_IRQn);

    HAL_SPI_DeInit(&boardCommSPIHandle);
    HAL_SPI_Init(&boardCommSPIHandle);

    /* Re-enable SPI2 and DMA channels. */
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_RX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_TX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_SPI->CR1 |= SPI_CR1_SPE;
}