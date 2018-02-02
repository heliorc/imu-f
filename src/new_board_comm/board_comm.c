#include "includes.h"
#include "board_comm.h"

//board_comm spi stuff lives here
SPI_InitTypeDef boardCommSpiInitStruct;
DMA_InitTypeDef boardCommDmaInitStruct;

#define SPI_BUFFER_SIZE 64
uint8_t boardCommRxBuff[SPI_BUFFER_SIZE];
uint8_t boardCommTxBuff[SPI_BUFFER_SIZE];

void board_comm_init(void)
{
    // setup board_comm spi mappings and gpio init
    single_gpio_init(BOARD_COMM_MISO_PORT, BOARD_COMM_MISO_PIN_SRC, BOARD_COMM_MISO_PIN, BOARD_COMM_MISO_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(BOARD_COMM_MOSI_PORT, BOARD_COMM_MOSI_PIN_SRC, BOARD_COMM_MOSI_PIN, BOARD_COMM_MOSI_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(BOARD_COMM_SCK_PORT,  BOARD_COMM_SCK_PIN_SRC,  BOARD_COMM_SCK_PIN,  BOARD_COMM_SCK_ALTERNATE,  GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    //setup NSS GPIO if need be then init SPI and DMA for the SPI based on NSS type
    if(BOARD_COMM_CS_MODE == NSS_HARD)
    {
        single_gpio_init(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN_SRC, BOARD_COMM_CS_PIN, BOARD_COMM_CS_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Hard);
    }
    else if(BOARD_COMM_CS_MODE == NSS_SOFT)
    {
        single_gpio_init(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN_SRC, BOARD_COMM_CS_PIN, BOARD_COMM_CS_ALTERNATE, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Soft);
    }
    else
    {
        spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Soft);
    }

    //start the SPI
    memset(boardCommTxBuff, 0, sizeof(boardCommTxBuff));
    memset(boardCommRxBuff, 0, sizeof(boardCommRxBuff));
    spi_fire_dma(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, &boardCommDmaInitStruct, SPI_BUFFER_SIZE, boardCommTxBuff, boardCommRxBuff);

    //call this when the transfer is done
    //cleanup_spi(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, &BOARD_COMM_TX_DMA_FLAG_GL, &BOARD_COMM_RX_DMA_FLAG_GL);
}

