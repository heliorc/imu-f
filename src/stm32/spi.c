#include "includes.h"
#include "board_comm.h"


//volatile spi_callback_function_pointer spiCallbackFunctionArray[3] = {0,};
//volatile spi_irq_callback_function_pointer spiIrqCallbackFunctionArray[3] = {0,};

//setup the DMA
void spi_init(SPI_InitTypeDef *spiInitStructure, DMA_InitTypeDef *dmaInitStructure, SPI_TypeDef *spi, uint16_t mode, uint16_t nss)
{
    SPI_I2S_DeInit(spi);
    spiInitStructure->SPI_Mode = mode;
    spiInitStructure->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInitStructure->SPI_DataSize = SPI_DataSize_8b;
    spiInitStructure->SPI_CPOL = SPI_CPOL_Low;
    spiInitStructure->SPI_CPHA = SPI_CPHA_1Edge;
    spiInitStructure->SPI_NSS = nss;
    spiInitStructure->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    spiInitStructure->SPI_FirstBit = SPI_FirstBit_MSB;
    spiInitStructure->SPI_CRCPolynomial = 7;
    SPI_Init(spi, spiInitStructure);
    //SPI_RxFIFOThresholdConfig(spi, SPI_RxFIFOThreshold_QF); 
  

    dmaInitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStructure->DMA_MemoryDataSize =  DMA_MemoryDataSize_Byte;
    dmaInitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStructure->DMA_Mode = DMA_Mode_Normal;
    dmaInitStructure->DMA_M2M = DMA_M2M_Disable;

    SPI_Cmd(spi, ENABLE);

}

//start the dma transaction
void spi_fire_dma(SPI_TypeDef *spi, DMA_Channel_TypeDef *txDma, DMA_Channel_TypeDef *rxDma, DMA_InitTypeDef *dmaInitStructure, uint32_t size, uint8_t *txBuff, uint8_t *rxBuff)
{
    //These two blocks of code are kind of expensive, we can make them much smaller and we need to for runtime.
    //setting the init structure piece by peice just to fill some regs is pricey when we can just set the regs ourselves
    //DMA channel Rx of SPI Configuration
    dmaInitStructure->DMA_BufferSize = size;
    dmaInitStructure->DMA_PeripheralBaseAddr = (uint32_t)(spi->DR);
    dmaInitStructure->DMA_MemoryBaseAddr = (uint32_t)rxBuff;
    dmaInitStructure->DMA_DIR = DMA_DIR_PeripheralSRC;
    dmaInitStructure->DMA_Priority = DMA_Priority_High;
    DMA_Init(rxDma, dmaInitStructure); //comp

    //setting the init structure piece by peice just to fill some regs is pricey when we can just set the regs ourselves
    //DMA channel Tx of SPI Configuration
    dmaInitStructure->DMA_BufferSize = size;
    dmaInitStructure->DMA_PeripheralBaseAddr = (uint32_t)(spi->DR);
    dmaInitStructure->DMA_MemoryBaseAddr = (uint32_t)txBuff;
    dmaInitStructure->DMA_DIR = DMA_DIR_PeripheralDST;
    dmaInitStructure->DMA_Priority = DMA_Priority_Low;
    DMA_Init(txDma, dmaInitStructure); //comp

    /* Enable the SPI Rx and Tx DMA requests */
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Rx, ENABLE); //simp
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Tx, ENABLE); //simp

    /* Enable the SPI peripheral */
    SPI_Cmd(spi, ENABLE); //simp

    /* Enable the DMA channels */
    DMA_Cmd(rxDma, ENABLE); //simp
    DMA_Cmd(txDma, ENABLE); //simp
}

//after spi transaction is done
void cleanup_spi(SPI_TypeDef *spi, DMA_Channel_TypeDef *txDma, DMA_Channel_TypeDef *rxDma, uint32_t *txDmaFlag, uint32_t *rxDmaFlag)
{
    /* Clear DMA1 global flags */
    DMA_ClearFlag(*txDmaFlag); //simp
    DMA_ClearFlag(*rxDmaFlag); //simp
   
    /* Disable the DMA channels */
    DMA_Cmd(txDma, DISABLE); //simp
    DMA_Cmd(rxDma, DISABLE); //simp
    
    /* Disable the SPI peripheral */
    SPI_Cmd(spi, DISABLE); //simp
   
    /* Disable the SPI Rx and Tx DMA requests */
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Tx, DISABLE); //simp
    SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Rx, DISABLE); //simp
}