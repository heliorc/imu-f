#include "includes.h"
#include "gyro.h"
#include "board_commm.h"

//SPI 2 is for the gyro
SPI_HandleTypeDef gyroSPIHandle;
DMA_HandleTypeDef hdmaGyroSPIRx;
DMA_HandleTypeDef hdmaGyroSPITx;
char gyroSpiRxBuffer[256];
char gyroSpiTxBuffer[256];

//SPI 3 is for the f4/f3
SPI_HandleTypeDef boardCommSPIHandle;
DMA_HandleTypeDef hdmaBoardCommSPIRx;
DMA_HandleTypeDef hdmaBoardCommSPITx;
char boardCommSpiRxBuffer[256];
char boardCommSpiTxBuffer[256];

volatile spi_callback_function_pointer spiCallbackFunctionArray[3] = {0,};

static void init_handle(SPI_HandleTypeDef* spiHandle, IRQn_Type irq);

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI1)
    {
		callbackFunctionArray[0](&hspi);
    }
    else if(hspi->Instance == SPI2)
    {
		callbackFunctionArray[1](&hspi);
    }
    else if(hspi->Instance == SPI3)
    {
		callbackFunctionArray[2](&hspi);
    }

    //if(hspi->Instance == BOARD_COMM_SPI)
    //{
        //spiCallbackFunctionArray[3];
        //#ifndef C3PUBL
        //memcpy(&newCommand, rxBuffer, sizeOf(bootloaderCommand_t));
        //if (newCommand.command && newCommand.command == newCommand.crc){
        //    run_command(&newCommand);
        //}
        //memset(rxBuffer, 0, 256);
        //HAL_SPI_TransmitReceive_IT(&MESSAGE_HANDLE, txData, rxData, 256);
        //#else
        //#endif
        //HAL_SPI_TransmitReceive_IT(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, 256);
    //}
    //else if(hspi->Instance == GYRO_SPI)
    //{

    //}
}




static void init_handle(SPI_HandleTypeDef* spiHandle, IRQn_Type irq)
{
    HAL_SPI_DeInit(spiHandle);
    HAL_NVIC_DisableIRQ(irq);

    if (HAL_SPI_Init(spiHandle) != HAL_OK)
    {
        //TODO: handle this error.
        while(1);
    }
}

void spi_init(SPI_HandleTypeDef* spiHandle, SPI_TypeDef* instance, uint32_t baudscaler, uint32_t spi_mode, uint32_t irqn, uint32_t irqp, uint32_t irqsp)
{
    /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
    spiHandle->Instance               = instance;
    spiHandle->Init.Mode              = spi_mode;
    spiHandle->Init.BaudRatePrescaler = baudscaler;
    spiHandle->Init.Direction         = SPI_DIRECTION_2LINES;
    spiHandle->Init.CLKPhase          = SPI_PHASE_2EDGE;
    spiHandle->Init.CLKPolarity       = SPI_POLARITY_HIGH;
    spiHandle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    spiHandle->Init.CRCPolynomial     = 7;
    spiHandle->Init.DataSize          = SPI_DATASIZE_8BIT;
    spiHandle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    spiHandle->Init.NSS               = SPI_NSS_SOFT;
    spiHandle->Init.TIMode            = SPI_TIMODE_DISABLE;
    spiHandle->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    spiHandle->Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
    init_handle((spiHandle), irqn);

    HAL_NVIC_SetPriority(SPI1_IRQn, irqp, irqsp);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

void spi_dma_init(SPI_HandleTypeDef* spiHandle, DMA_HandleTypeDef* hdma_spi_rx, DMA_HandleTypeDef* hdma_spi_tx, DMA_Channel_TypeDef rxDmaChannel, DMA_Channel_TypeDef txDmaChannel)
{

    (*hdma_spi_rx).Instance = rxDmaChannel;
    (*hdma_spi_rx).Init.Direction = DMA_PERIPH_TO_MEMORY;
    (*hdma_spi_rx).Init.PeriphInc = DMA_PINC_DISABLE;
    (*hdma_spi_rx).Init.MemInc = DMA_MINC_ENABLE;
    (*hdma_spi_rx).Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    (*hdma_spi_rx).Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    (*hdma_spi_rx).Init.Mode = DMA_NORMAL;
    (*hdma_spi_rx).Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(hdma_spi_rx) != HAL_OK)
    {
        //todo error handler
        while(1);
    }

    __HAL_LINKDMA(spiHandle,hdmarx,*hdma_spi_rx);

    /* SPI3_TX Init */
    (*hdma_spi_tx).Instance = txDmaChannel;
    (*hdma_spi_tx).Init.Direction = DMA_MEMORY_TO_PERIPH;
    (*hdma_spi_tx).Init.PeriphInc = DMA_PINC_DISABLE;
    (*hdma_spi_tx).Init.MemInc = DMA_MINC_ENABLE;
    (*hdma_spi_tx).Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    (*hdma_spi_tx).Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    (*hdma_spi_tx).Init.Mode = DMA_NORMAL;
    (*hdma_spi_tx).Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(hdma_spi_tx) != HAL_OK)
    {
        //todo error handler
        while(1);
    }

    __HAL_LINKDMA(spiHandle,hdmatx,*hdma_spi_tx);

}