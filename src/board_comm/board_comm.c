#include "includes.h"
#include "board_comm.h"
#include "gyro.h"
#include "fast_kalman.h"
#include "filter.h"
#include "crc.h"

//board_comm spi stuff lives here, actually, it all should probably go under boardCommState_t
SPI_InitTypeDef boardCommSpiInitStruct;
DMA_InitTypeDef boardCommDmaInitStruct;
volatile imufCommand_t bcRx;
volatile imufCommand_t bcTx;
volatile uint8_t* bcRxPtr;
volatile uint8_t* bcTxPtr;
volatile uint32_t spiDoneFlag;
volatile boardCommState_t boardCommState;
volatile uint32_t filterMode;

static void run_command(volatile imufCommand_t* command, volatile imufCommand_t* reply);

void clear_imuf_command(volatile imufCommand_t* command)
{
    memset((uint8_t*)command, 0, sizeof(imufCommand_t));
}

void board_comm_init(void)
{
    //set comm size based on size of structure
    boardCommState.bufferSize = sizeof(imufCommand_t) - 2; //last word is for overflow, the syncWord
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
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Soft, SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_BaudRatePrescaler_2); 
    #else
        single_gpio_init(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN_SRC, BOARD_COMM_CS_PIN, BOARD_COMM_CS_ALTERNATE, BOARD_COMM_CS_TYPE, GPIO_OType_PP, GPIO_PuPd_NOPULL);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, BOARD_COMM_CS_TYPE, SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_BaudRatePrescaler_2);
    #endif

    single_gpio_init(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN_SRC, BOARD_COMM_DATA_RDY_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);

}

int parse_imuf_command(volatile imufCommand_t* command)
{
    if ( command->command && command->crc == get_crc( (volatile uint32_t*)command, 11) )
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
    append_crc_to_data_v((volatile uint32_t *)bcTxPtr, 11); //11 will put the crc at the location it needs to be which is imufCommand.crc
    //this takes 0.78us to run
    cleanup_spi(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, BOARD_COMM_SPI_RST_MSK);
    //this takes 1.19us to run
    spi_fire_dma(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, &boardCommDmaInitStruct, (uint32_t *)&(boardCommState.bufferSize), bcTxPtr, bcRxPtr);
    gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
}

inline void board_comm_spi_complete(void)
{
    spiDoneFlag = 1;
    gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
}

void board_comm_spi_callback_function(void)
{

    board_comm_spi_complete(); //this needs to be called when the transaction is complete

    if ( ( (boardCommState.commMode != GTBCM_SETUP) || (bcTx.command == BC_IMUF_LISTENING) ) && parse_imuf_command(&bcRx) )//we  were waiting for a command //we have a valid command
    {
        //command checks out
        //run the command and generate the reply
        run_command(&bcRx,&bcTx); 
        
    }
    else if ( (bcTx.command == BC_IMUF_SETUP) ) //we just replied that we got proper setup commands, let's activate them now
    {
        bcTx.command = 0;
        //set flight mode now
        boardCommState.bufferSize = filterMode;
        boardCommState.commMode   = filterMode;
        allow_filter_init();
        reset_matrix(); //reset orientation matrix in case it's been changes
        reset_loop(); //set loop speed
    }
    else 
    {
        //bad command, listen for another
        bcTx.command = BC_IMUF_LISTENING;
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
                reply->command = BC_IMUF_CALIBRATE;
            }
            calibratingGyro=1;
        break;
        case BC_IMUF_REPORT_INFO:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                memcpy((uint8_t*)&(reply->param1), (uint8_t*)&flightVerson, sizeof(flightVerson));
                reply->command = BC_IMUF_REPORT_INFO;
            }
        break;
        case BC_IMUF_SETUP:
            if(boardCommState.commMode == GTBCM_SETUP) //can only send reply if we're not in runtime
            {
                filterMode                       = (command->param1);
                gyroSettingsConfig.rate          = (command->param2 >> 16);
                filterConfig.i_pitch_q           = (command->param3 >> 16);
                filterConfig.i_roll_q            = (command->param4 >> 16);
                filterConfig.i_yaw_q             = (command->param5 >> 16);
                filterConfig.filterWindow[PITCH] = (command->param3 & 0xFFFF);
                filterConfig.filterWindow[ROLL]  = (command->param4 & 0xFFFF);
                filterConfig.filterWindow[YAW]   = (command->param5 & 0xFFFF);
                filterConfig.i_pitch_lpf_hz      = (command->param6 >> 16);
                filterConfig.i_roll_lpf_hz       = (command->param6 & 0xFFFF);
                filterConfig.i_yaw_lpf_hz        = (command->param7 >> 16);
                gyroSettingsConfig.orientation   = (uint32_t)((uint16_t)(command->param8 & 0xFFFF));
                gyroSettingsConfig.smallX        = (int32_t)(  (int16_t)(command->param8 >> 16));
                gyroSettingsConfig.smallY        = (int32_t)(  (int16_t)(command->param9 & 0xFFFF));
                gyroSettingsConfig.smallZ        = (int32_t)(  (int16_t)(command->param9 >> 16));

                memset((uint8_t *)reply, 0, sizeof(imufCommand_t));
                reply->command = BC_IMUF_SETUP;
            }
        break;
        default:
        break;
    }
}