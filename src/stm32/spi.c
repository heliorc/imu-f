#include "includes.h"
#include "board_comm.h"


volatile spi_tx_done_callback spiCallbackFunctionArray[3] = {0,};
//volatile spi_callback_function_pointer spiCallbackFunctionArray[3] = {0,};
//volatile spi_irq_callback_function_pointer spiIrqCallbackFunctionArray[3] = {0,};

//setup the DMA
void spi_init(SPI_InitTypeDef *spiInitStructure, DMA_InitTypeDef *dmaInitStructure, SPI_TypeDef *spi, uint16_t mode, uint16_t nss, uint16_t cpol, uint16_t cpha, uint16_t BaudRatePrescaler)
{
    SPI_I2S_DeInit(spi);
    spiInitStructure->SPI_Mode = mode;
    spiInitStructure->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInitStructure->SPI_DataSize = SPI_DataSize_8b;
    spiInitStructure->SPI_CPOL = cpol;
    spiInitStructure->SPI_CPHA = cpha;
    spiInitStructure->SPI_NSS = nss;
    spiInitStructure->SPI_BaudRatePrescaler = BaudRatePrescaler;
    spiInitStructure->SPI_FirstBit = SPI_FirstBit_MSB;
    spiInitStructure->SPI_CRCPolynomial = 7;
    SPI_Init(spi, spiInitStructure);
    SPI_RxFIFOThresholdConfig(spi,SPI_RxFIFOThreshold_QF);
  
    //set DMA to default state
    DMA_DeInit(BOARD_COMM_TX_DMA);
    DMA_DeInit(BOARD_COMM_RX_DMA);

    dmaInitStructure->DMA_M2M = DMA_M2M_Disable;
    dmaInitStructure->DMA_Mode = DMA_Mode_Normal;
    dmaInitStructure->DMA_Priority = DMA_Priority_Low;
    dmaInitStructure->DMA_DIR = DMA_DIR_PeripheralDST;

    dmaInitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStructure->DMA_PeripheralBaseAddr = (uint32_t)&BOARD_COMM_SPI->DR;

    dmaInitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStructure->DMA_MemoryBaseAddr = 0; //this is set later when we fire the DMA

    dmaInitStructure->DMA_BufferSize = 1;     //this is set later when we fire the DMA

    DMA_Init(BOARD_COMM_TX_DMA, dmaInitStructure);

    dmaInitStructure->DMA_Priority = DMA_Priority_Medium;
    dmaInitStructure->DMA_DIR = DMA_DIR_PeripheralSRC;

    DMA_Init(BOARD_COMM_RX_DMA, dmaInitStructure);

    SPI_Cmd(spi, ENABLE);

}

//start the dma transaction
void spi_fire_dma(SPI_TypeDef *spi, DMA_Channel_TypeDef *txDma, DMA_Channel_TypeDef *rxDma, DMA_InitTypeDef *dmaInitStructure, uint32_t *size, volatile uint8_t *txBuff, volatile uint8_t *rxBuff)
{
    //set buffer size
    DMA_SetCurrDataCounter(txDma, *size);
    DMA_SetCurrDataCounter(rxDma, *size);

    //set buffer
    txDma->CMAR = (uint32_t)txBuff;
    rxDma->CMAR = (uint32_t)rxBuff;

    //enable DMA SPI streams
    DMA_Cmd(txDma, ENABLE);
    DMA_Cmd(rxDma, ENABLE);

    //enable DMA SPI requests
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Rx, ENABLE);

    //enable and send
    SPI_Cmd(spi, ENABLE);
}

//after spi transaction is done
void cleanup_spi(SPI_TypeDef *spi, DMA_Channel_TypeDef *txDma, DMA_Channel_TypeDef *rxDma, uint32_t resetMask)
{
    //clear DMA flags
    DMA_ClearFlag(BOARD_COMM_ALL_DMA_FLAGS);

    //disable DMAs
    DMA_Cmd(txDma,DISABLE);
    DMA_Cmd(rxDma,DISABLE);  

    //disable SPI DMA requests
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Tx, DISABLE);
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Rx, DISABLE);

    // Reset SPI (clears TXFIFO).
    RCC->APB1RSTR |= resetMask;
    RCC->APB1RSTR &= ~resetMask;

    //disable SPI
    SPI_Cmd(spi, DISABLE);
}

int spi_transfer_blocking(SPI_TypeDef* spi, const uint8_t* txBuff, uint8_t* rxBuff, int len)
{
    //TODO: Check timeout
    while (len--)
    {
        while(SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET); //tx buffer empyt?
        SPI_SendData8(spi, *(txBuff++));
        while( spi->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
        *(rxBuff++) = SPI_ReceiveData8(spi);
    }

    return 1;
}