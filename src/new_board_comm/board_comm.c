#include "includes.h"
#include "board_comm.h"

//board_comm spi stuff lives here
SPI_InitTypeDef boardCommSpiInitStruct;
DMA_InitTypeDef boardCommDmaInitStruct;
volatile imufCommand_t bcRx;
volatile imufCommand_t bcTx;
volatile uint8_t* bcRxPtr;
volatile uint8_t* bcTxPtr;
volatile uint32_t spiDoneFlag;
uint32_t boardCommSize;

void clear_imuf_command(volatile imufCommand_t* command)
{
    //clear a volatile structure using 32 bit pointer typecast
    //for (int x=sizeof(imufCommand_t)-sizeof(uint32_t); x>=0; x-=sizeof(uint32_t))
    //{
    //    (* ( ((volatile uint32_t *)command) + x) ) = 0;
    //}
    memset((uint8_t*)command, 0, sizeof(imufCommand_t));
}

void volatile_uint32_copy(volatile uint32_t* dst, volatile uint32_t* src, uint32_t size)
{
    //clear a volatile structure using 32 bit pointer typecast
    //for (int x=size-sizeof(uint32_t); x>=0; x-=sizeof(uint32_t))
    //{
    //    ((dst + x)) = (*(src + x));
    //}
    memcpy((uint8_t*)dst, (uint8_t*)src, size);
}

void board_comm_init(void)
{
    //set comm size based on size of structure
    boardCommSize = sizeof(imufCommand_t) - sizeof(uint32_t); //last word is for overflow, the syncWord
    //boardCommSize = sizeof(imufCommand_t); //last word is for overflow, the syncWord

    //set uint8_t pointer to avoid casting each time
    bcRxPtr = (volatile uint8_t *)&bcRx;
    bcTxPtr = (volatile uint8_t *)&bcTx;

    // setup board_comm spi mappings and gpio init
    single_gpio_init(BOARD_COMM_MISO_PORT, BOARD_COMM_MISO_PIN_SRC, BOARD_COMM_MISO_PIN, BOARD_COMM_MISO_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(BOARD_COMM_MOSI_PORT, BOARD_COMM_MOSI_PIN_SRC, BOARD_COMM_MOSI_PIN, BOARD_COMM_MOSI_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(BOARD_COMM_SCK_PORT,  BOARD_COMM_SCK_PIN_SRC,  BOARD_COMM_SCK_PIN,  BOARD_COMM_SCK_ALTERNATE,  GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);

//single_gpio_init(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PORT_SRC, BOARD_COMM_EXTI_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
//gpio_write_pin(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PIN, 0);
//gpio_write_pin(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PIN, 1);
//gpio_write_pin(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PIN, 0);
//gpio_write_pin(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PIN, 1);
    //setup NSS GPIO if need be then init SPI and DMA for the SPI based on NSS type
    if(BOARD_COMM_CS_TYPE == NSS_HARD)
    {
        single_gpio_init(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN_SRC, BOARD_COMM_CS_PIN, BOARD_COMM_CS_ALTERNATE, BOARD_COMM_CS_TYPE, GPIO_OType_PP, GPIO_PuPd_NOPULL);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Hard);
    }
    else if(BOARD_COMM_CS_TYPE == NSS_SOFT)
    {
        single_gpio_init(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN_SRC, BOARD_COMM_CS_PIN, BOARD_COMM_CS_ALTERNATE, BOARD_COMM_CS_TYPE, GPIO_OType_PP, GPIO_PuPd_NOPULL);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Soft);
    }
    else
    {
        //exti is used for NSS
        gpio_exti_init(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PORT_SRC, BOARD_COMM_EXTI_PIN, BOARD_COMM_EXTI_PIN_SRC, BOARD_COMM_EXTI_LINE, EXTI_Trigger_Rising, BOARD_COMM_EXTI_IRQn, BOARD_COMM_EXTI_ISR_PRE_PRI, BOARD_COMM_EXTI_ISR_SUB_PRI);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Soft);
    }

    
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
    spiDoneFlag = 0;

    spi_fire_dma(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, &boardCommDmaInitStruct, &boardCommSize, (uint8_t *)&bcTx, (uint8_t *)&bcRx);
    gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
}

void board_comm_spi_complete(void)
{
    gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
    cleanup_spi(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA_FLAG_GL, BOARD_COMM_RX_DMA_FLAG_GL);
}