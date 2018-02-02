#include "includes.h"

SPI_InitTypeDef  SPI_InitStructure;
DMA_InitTypeDef  DMA_InitStructure;

#define SPI_BUFFER_SIZE 64
uint8_t spiRxBuffer[SPI_BUFFER_SIZE];
uint8_t spiTxBuffer[SPI_BUFFER_SIZE];

void board_init(void)
{
    clock_config();
    board_gpio_init();
}

void spi_done_callback(void)
{
    /* Clear DMA1 global flags */
    DMA_ClearFlag(BOARD_COMM_TX_DMA_FLAG); //simp
    DMA_ClearFlag(BOARD_COMM_RX_DMA_FLAG); //simp
   
    /* Disable the DMA channels */
    DMA_Cmd(BOARD_COMM_RX_DMA, DISABLE); //simp
    DMA_Cmd(BOARD_COMM_TX_DMA, DISABLE); //simp
    
    /* Disable the SPI peripheral */
    SPI_Cmd(BOARD_COMM_SPI, DISABLE); //simp
   
    /* Disable the SPI Rx and Tx DMA requests */
    SPI_I2S_DMACmd(BOARD_COMM_SPI, SPI_I2S_DMAReq_Rx, DISABLE); //simp
    SPI_I2S_DMACmd(BOARD_COMM_SPI, SPI_I2S_DMAReq_Tx, DISABLE); //simp
}

void spi_rewind_dma(void)
{
    /* DMA channel Rx of SPI Configuration */
    DMA_InitStructure.DMA_BufferSize = SPI_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(BOARD_COMM_SPI->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &spiRxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(BOARD_COMM_RX_DMA, &DMA_InitStructure);

    /* DMA channel Tx of SPI Configuration */
    DMA_InitStructure.DMA_BufferSize = SPI_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(BOARD_COMM_SPI->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &spiTxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_Init(BOARD_COMM_TX_DMA, &DMA_InitStructure);

    /* Enable the SPI Rx and Tx DMA requests */
    SPI_I2S_DMACmd(BOARD_COMM_SPI, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(BOARD_COMM_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

    /* Enable the SPI peripheral */
    SPI_Cmd(BOARD_COMM_SPI, ENABLE);

    /* Enable the DMA channels */
    DMA_Cmd(BOARD_COMM_RX_DMA, ENABLE);
    DMA_Cmd(BOARD_COMM_TX_DMA, ENABLE);
}

void spi_init(void)
{
    SPI_I2S_DeInit(BOARD_COMM_SPI);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_BUFFER_SIZE;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(BOARD_COMM_SPI, &SPI_InitStructure);
    //SPI_RxFIFOThresholdConfig(BOARD_COMM_SPI, SPI_RxFIFOThreshold_QF); 
  

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize =  DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    SPI_Cmd(BOARD_COMM_SPI, ENABLE);

}

int main(void)
{
    board_init();
    spi_rewind_dma();
    spi_init();

    while(1)
    {
        if (spiRxBuffer[3] == 'r' )
        {
            spiTxBuffer[3] = 'r';
            spiRxBuffer[3] = 0;
            spi_done_callback();
        }
    }
}