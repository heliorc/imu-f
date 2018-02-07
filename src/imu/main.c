#include "includes.h"
#include "board_comm.h"
#include "quaternions.h"
#include "imu.h"
#include "gyro.h"


int main(void)
{

    board_init();       //inits the clocks  



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
    //SPI_RxFIFOThresholdConfig(spi, SPI_RxFIFOThreshold_QF); 
  

    dmaInitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStructure->DMA_MemoryDataSize =  DMA_MemoryDataSize_Byte;
    dmaInitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStructure->DMA_Mode = DMA_Mode_Normal;
    dmaInitStructure->DMA_M2M = DMA_M2M_Disable;

    SPI_Cmd(spi, ENABLE);
}