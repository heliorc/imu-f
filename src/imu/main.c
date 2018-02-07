#include "includes.h"
#include "board_comm.h"
#include "quaternions.h"
#include "imu.h"
#include "gyro.h"


uint8_t buff1[2];
uint8_t buff2[2];

void spi_init_test(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Configure SPI pins: CS
    GPIO_InitStructure.GPIO_Pin = GYRO_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GYRO_CS_PORT, &GPIO_InitStructure);

    GPIO_SetBits(GYRO_CS_PORT, GYRO_CS_PIN); // Drive CS high

    // Configure SPI pins: SCK
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;     // Now that CS GPIO has been initialized to Mode: OUT, change GPIO Mode to AF
    GPIO_InitStructure.GPIO_Pin = GYRO_SCK_PIN;
    GPIO_Init(GYRO_SCK_PORT, &GPIO_InitStructure);

    // Configure SPI pins: MISO
    GPIO_InitStructure.GPIO_Pin = GYRO_MISO_PIN;
    GPIO_Init(GYRO_MISO_PORT, &GPIO_InitStructure);

    // Configure SPI pins: MOSI
    GPIO_InitStructure.GPIO_Pin = GYRO_MOSI_PIN;
    GPIO_Init(GYRO_MOSI_PORT, &GPIO_InitStructure);

    // Configure alternate function to SPI related GPIOs to act as SPI peripheral
    GPIO_PinAFConfig(GYRO_SCK_PORT, GYRO_SCK_PIN_SRC, GYRO_SCK_ALTERNATE);
    GPIO_PinAFConfig(GYRO_MISO_PORT, GYRO_MISO_PIN_SRC, GYRO_MISO_ALTERNATE);
    GPIO_PinAFConfig(GYRO_MOSI_PORT, GYRO_MOSI_PIN_SRC, GYRO_MOSI_ALTERNATE);

    // Configure SPI
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(GYRO_SPI, &SPI_InitStructure);
    SPI_RxFIFOThresholdConfig(GYRO_SPI,SPI_RxFIFOThreshold_QF);


    DMA_DeInit(GYRO_TX_DMA);
    DMA_DeInit(GYRO_RX_DMA);

    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;               // M2M Disabled- Peripheral mode (requires timer trigger)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;              // Normal mode
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      // Medium priority
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;         // Memory to Peripheral

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;           // 8-bit Register
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // Always write to same register
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GYRO_SPI->DR;       // Output data for SPI peripheral

    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                   // 8-bit array
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // Increment through array
    DMA_InitStructure.DMA_MemoryBaseAddr = 0;                                                              // Initialize later

    DMA_InitStructure.DMA_BufferSize = 1;                                                                                     // Initialize later

    DMA_Init(GYRO_TX_DMA, &DMA_InitStructure);            // Initialize TX DMA

    // Initialize RX DMA
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;          // Peripheral to Memory

    DMA_Init(GYRO_RX_DMA, &DMA_InitStructure);            // Initialize RX DMA
}

void cs_lo(void)
{
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 0);
}

void cs_hi(void)
{
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 1);
}

void spi_tran_test(void)
{     
    buff1[0] = 245; //who am i?
    buff1[1] = 0;
    buff2[0] = 0;
    buff2[1] = 0;

    DMA_SetCurrDataCounter(GYRO_TX_DMA, 2);
    DMA_SetCurrDataCounter(GYRO_RX_DMA, 2);

    // Configure the peripheral base address
    GYRO_TX_DMA->CMAR = (uint32_t)buff1;
    GYRO_RX_DMA->CMAR = (uint32_t)buff2;

    /* The Data transfer is performed in the SPI using Direct Memory Access */

    /* Enable DMA SPI TX Stream */
    DMA_Cmd(GYRO_TX_DMA, ENABLE);

    /* Enable DMA SPI RX Stream */
    DMA_Cmd(GYRO_RX_DMA, ENABLE);

    // Assert the CS
    cs_lo();

    /* Enable SPI DMA TX Requsts */
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Tx, ENABLE);

    /* Enable SPI DMA RX Requsts */
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

    /* Enable the SPI peripheral */
    SPI_Cmd(GYRO_SPI, ENABLE);

    /* Waiting the end of Data transfer */
    volatile unsigned int timeoutCounter = 0;
    while ((DMA_GetFlagStatus(GYRO_TX_DMA_FLAG_TC)==RESET) && (timeoutCounter < 200000))
    {
        timeoutCounter++;
    }
    timeoutCounter = 0;

    while ((DMA_GetFlagStatus(GYRO_RX_DMA_FLAG_TC)==RESET) && (timeoutCounter < 200000))
    {
        timeoutCounter++;
    }

    /* Clear DMA Flags */
    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_HT2 | DMA1_FLAG_TC2 | DMA1_FLAG_GL3 | DMA1_FLAG_HT3 | DMA1_FLAG_TC3);

    /* Disable DMA SPI TX Stream */
    DMA_Cmd(GYRO_TX_DMA,DISABLE);

    /* Disable DMA SPI RX Stream */
    DMA_Cmd(GYRO_RX_DMA,DISABLE);  

    /* Disable SPI DMA TX Requsts */
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

    /* Disable SPI DMA RX Requsts */
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

    /* Disable the SPI peripheral */
    SPI_Cmd(GYRO_SPI, DISABLE);

    // Release the CS
    cs_hi();
}

int main(void)
{

    board_init();       //inits the clocks  

    spi_init_test();
    delay_ms(5);
    spi_tran_test();
}