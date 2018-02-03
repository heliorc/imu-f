#include "includes.h"
#include "board_comm.h"

//board_comm spi stuff lives here, actually, it all should probably go under boardCommState_t
SPI_InitTypeDef boardCommSpiInitStruct;
DMA_InitTypeDef boardCommDmaInitStruct;
volatile imufCommand_t bcRx;
volatile imufCommand_t bcTx;
volatile uint8_t* bcRxPtr;
volatile uint8_t* bcTxPtr;
volatile uint32_t spiDoneFlag;
volatile boardCommState_t boardCommState;

static void run_command(volatile imufCommand_t* command, volatile imufCommand_t* reply);

void clear_imuf_command(volatile imufCommand_t* command)
{
    memset((uint8_t*)command, 0, sizeof(imufCommand_t));
}

void board_comm_init(void)
{
    //set comm size based on size of structure
    boardCommState.bufferSize = sizeof(imufCommand_t) - sizeof(uint32_t); //last word is for overflow, the syncWord
    boardCommState.commMode   = GTBCM_SETUP;

    //set uint8_t pointer to avoid casting each time
    bcRxPtr = (volatile uint8_t *)&bcRx;
    bcTxPtr = (volatile uint8_t *)&bcTx;

    // setup board_comm spi mappings and gpio init
    single_gpio_init(BOARD_COMM_MISO_PORT, BOARD_COMM_MISO_PIN_SRC, BOARD_COMM_MISO_PIN, BOARD_COMM_MISO_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(BOARD_COMM_MOSI_PORT, BOARD_COMM_MOSI_PIN_SRC, BOARD_COMM_MOSI_PIN, BOARD_COMM_MOSI_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(BOARD_COMM_SCK_PORT,  BOARD_COMM_SCK_PIN_SRC,  BOARD_COMM_SCK_PIN,  BOARD_COMM_SCK_ALTERNATE,  GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    //setup NSS GPIO if need be then init SPI and DMA for the SPI based on NSS type
    #ifndef BOARD_COMM_CS_TYPE
        //exti is used for NSS
        gpio_exti_init(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PORT_SRC, BOARD_COMM_EXTI_PIN, BOARD_COMM_EXTI_PIN_SRC, BOARD_COMM_EXTI_LINE, EXTI_Trigger_Rising, BOARD_COMM_EXTI_IRQn, BOARD_COMM_EXTI_ISR_PRE_PRI, BOARD_COMM_EXTI_ISR_SUB_PRI);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Soft); 
    #else
        single_gpio_init(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN_SRC, BOARD_COMM_CS_PIN, BOARD_COMM_CS_ALTERNATE, BOARD_COMM_CS_TYPE, GPIO_OType_PP, GPIO_PuPd_NOPULL);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, BOARD_COMM_CS_TYPE);
    #endif

    single_gpio_init(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN_SRC, BOARD_COMM_DATA_RDY_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);

}

int parse_imuf_command(volatile imufCommand_t* command)
{
    if (command->command && command->command == command->crc)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void start_listening(void)
{
    spiDoneFlag = 0; //flag for use during runtime to limit ISR overhead, might be able to remove this completely 
    //this takes 1.19us to run
    spi_fire_dma(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, &boardCommDmaInitStruct, (uint32_t *)&(boardCommState.bufferSize), bcTxPtr, bcRxPtr);
    gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
}

void board_comm_spi_complete(void)
{
    gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
    //this takes 0.78us to run
    cleanup_spi(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA_FLAG_GL, BOARD_COMM_RX_DMA_FLAG_GL, BOARD_COMM_SPI_RST_MSK);
}

void board_comm_spi_callback_function(void)
{
    board_comm_spi_complete(); //this needs to be called when the transaction is complete

    //calibration won't work this way
    if ( (bcTx.command == BC_IMUF_LISTENING) && parse_imuf_command(&bcRx) )//we  were waiting for a command //we have a valid command
    {
        //command checks out
        //run the command and generate the reply
        run_command(&bcRx,&bcTx); 
        
    }
    else if ( (bcTx.command == BC_IMUF_SETUP) && parse_imuf_command(&bcRx) ) //we just replied that we got proper setup commands, let's activate them now
    {
        //set flight mode now
        boardCommState.bufferSize = sizeof(imufCommand_t) - sizeof(uint32_t); //last word is for overflow, the syncWord
        boardCommState.commMode   = GTBCM_SETUP;
    }
    else 
    {
        //bad command, listen for another
        clear_imuf_command(&bcRx);
        clear_imuf_command(&bcTx);
        bcTx.command = bcTx.crc = BC_IMUF_LISTENING;
    }

    //start the process again, if still in setup mode
    if (boardCommState.commMode == GTBCM_SETUP)
    {
        start_listening();
    }

}

static void run_command(volatile imufCommand_t* command, volatile imufCommand_t* reply)
{
    switch (command->command)
    {
        case BC_IMUF_CALIBRATE:
            //might be good to make sure we're not flying when we run this command
            //this reply can only happen when in setup mode
            if(boardCommState.commMode == GTBCM_SETUP)
            {
                memset((uint8_t *)reply, 0, sizeof(imufCommand_t));
                reply->command = reply->crc = BC_IMUF_CALIBRATE;
            }
            //calibratingGyro=1; //comented out for now until it's added
        break;
        case BC_IMUF_REPORT_INFO:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                memcpy((uint8_t*)&(reply->param1), (uint8_t*)&flightVerson, sizeof(flightVerson));
                reply->command = reply->crc = BC_IMUF_REPORT_INFO;
            }
        break;
        case BC_IMUF_SETUP:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                //let's pretend the f4 sent this for now

                //commenting this out until the rest of the imu stuff is added
                //f4 needs to send all valid setup commands
                //uint32_t filterMode      = newCommand->param1;
                //uint32_t gyroOrientation = newCommand->param2;
                //We should NOT use floats here at all. We should change this so the only ISR with floats does the conversion
                //filterConfig.pitch_q  = ((float)(newCommand->param3 & 0xFFFF));
                //filterConfig.pitch_r  = ((float)(newCommand->param3 >> 16));
                //filterConfig.roll_q   = ((float)(newCommand->param4 & 0xFFFF));
                //filterConfig.roll_r   = ((float)(newCommand->param4 >> 16));
                //filterConfig.yaw_q    = ((float)(newCommand->param5 & 0xFFFF));
                //filterConfig.yaw_r    = ((float)(newCommand->param5 >> 16));
                //memset((uint8_t *)&imufCommandTx, 0, sizeof(imufCommandTx));
                //imufCommandTx.command = BC_IMUF_SETUP;
                //imufCommandTx.crc     = BC_IMUF_SETUP;
                memset((uint8_t *)reply, 0, sizeof(imufCommand_t));
                reply->command = reply->crc = BC_IMUF_SETUP;
            }
        break;
        default:
        break;
    }
}