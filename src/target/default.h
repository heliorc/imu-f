#pragma once
//springboard header file
#include "includes.h"

enum
{
    NSS_NONE = 0,
    NSS_HARD = 1,
    NSS_SOFT = 2,
};

#if defined(OMNIBUS)
    #include "omnibus.h"
#elif defined(MOTOLAB)
    #include "motolab.h"
#else

    #include "stm32f30x.h"

    #define FLASHSIZE_BASE                  ((uint32_t)0x1FFFF7CCU)         /*!< FLASH Size register base address */
    #define UID_BASE                        ((uint32_t)0x1FFFF7ACU)         /*!< Unique device ID register base address */
    #define FLASH_PAGE_SIZE                 0x800

    //clock config for std per
    #define TARGET_HSE                      ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON))
    #define TARGET_FLASH_LATENCY            0x4U
    #define TARGET_PLL_MUL                  RCC_CFGR_PLLMULL8
    #define TARGET_AHB_DIV                  (uint32_t)RCC_CFGR_HPRE_DIV1
    #define TARGET_APBH1_DIV                (uint32_t)RCC_CFGR_PPRE1_DIV2
    #define TARGET_APBH2_DIV                (uint32_t)RCC_CFGR_PPRE2_DIV1

    #define GYRO_SPI                        SPI3
    #define GYRO_SPI_NUM                    2   //SPI3 = 2, SPI2 = 1, SPI1 = 0,
    #define GYRO_SPI_IRQn                   SPI3_IRQn
    #define GYRO_RX_DMA                     DMA1_Channel2
    #define GYRO_TX_DMA                     DMA1_Channel3
    #define GYRO_CS_TYPE                    NSS_SOFT
    #define GYRO_CS_PIN_SRC                 GPIO_PinSource4
    #define GYRO_CS_PIN                     GPIO_Pin_4
    #define GYRO_CS_PORT                    GPIOA
    #define GYRO_CS_ALTERNATE               0
    #define GYRO_CS_MODE                    GPIO_MODE_OUTPUT_PP
    #define GYRO_MISO_PIN                   GPIO_PinSource4
    #define GYRO_MISO_PORT                  GPIOB
    #define GYRO_MISO_ALTERNATE             GPIO_AF_6
    #define GYRO_MOSI_PIN                   GPIO_PinSource5
    #define GYRO_MOSI_PORT                  GPIOB
    #define GYRO_MOSI_ALTERNATE             GPIO_AF_6
    #define GYRO_SCK_PIN                    GPIO_PinSource3
    #define GYRO_SCK_PORT                   GPIOB
    #define GYRO_SCK_ALTERNATE              GPIO_AF_6
    #define GYRO_SPI_RX_DMA_HANDLER         DMA1_Channel2_IRQHandler
    #define GYRO_SPI_TX_DMA_HANDLER         DMA1_Channel3_IRQHandler
    #define GYRO_SPI_RX_DMA_IRQn            DMA1_Channel2_IRQn
    #define GYRO_SPI_TX_DMA_IRQn            DMA1_Channel3_IRQn

    #define GYRO_EXTI_PORT                  GPIOA
    #define GYRO_EXTI_PORT_SRC              EXTI_PortSourceGPIOA
    #define GYRO_EXTI_PIN_SRC               EXTI_PinSource3
    #define GYRO_EXTI_PIN                   GPIO_Pin_3
    #define GYRO_EXTI_IRQn                  EXTI3_IRQn
    #define GYRO_EXTI_HANDLER               EXTI3_IRQHandler
    #define GYRO_EXTI_LINE                  EXTI_Line3


    #define BOARD_COMM_SPI                  SPI2
    #define BOARD_COMM_SPI_NUM              1   //SPI3 = 2, SPI2 = 1, SPI1 = 0,
    #define BOARD_COMM_SPI_IRQn             SPI2_IRQn
    #define BOARD_COMM_SPI_RST_MSK          RCC_APB1RSTR_SPI2RST
    #define BOARD_COMM_RX_DMA               DMA1_Channel4
    #define BOARD_COMM_TX_DMA               DMA1_Channel5
    #define BOARD_COMM_RX_DMA_FLAG_GL       DMA1_FLAG_GL4
    #define BOARD_COMM_TX_DMA_FLAG_GL       DMA1_FLAG_GL5
    #define BOARD_COMM_RX_DMA_FLAG_TC       DMA1_FLAG_TC4
    #define BOARD_COMM_TX_DMA_FLAG_TC       DMA1_FLAG_TC5

    #define BOARD_COMM_CS_TYPE              NSS_NONE
    #define BOARD_COMM_CS_PIN_SRC           GPIO_PinSource1
    #define BOARD_COMM_CS_PIN               GPIO_Pin_1
    #define BOARD_COMM_CS_PORT              GPIOA
    #define BOARD_COMM_CS_ALTERNATE         0
    #define BOARD_COMM_CS_MODE              GPIO_Mode_OUT

    #define BOARD_COMM_MISO_PIN_SRC         GPIO_PinSource10
    #define BOARD_COMM_MISO_PIN             GPIO_Pin_10
    #define BOARD_COMM_MISO_PORT            GPIOA
    #define BOARD_COMM_MISO_ALTERNATE       GPIO_AF_5

    #define BOARD_COMM_MOSI_PIN_SRC         GPIO_PinSource11
    #define BOARD_COMM_MOSI_PIN             GPIO_Pin_11
    #define BOARD_COMM_MOSI_PORT            GPIOA
    #define BOARD_COMM_MOSI_ALTERNATE       GPIO_AF_5

    #define BOARD_COMM_SCK_PIN_SRC          GPIO_PinSource1
    #define BOARD_COMM_SCK_PIN              GPIO_Pin_1
    #define BOARD_COMM_SCK_PORT             GPIOF
    #define BOARD_COMM_SCK_ALTERNATE        GPIO_AF_5

    #define BOARD_COMM_SPI_RX_DMA_HANDLER   DMA1_Channel4_IRQHandler
    #define BOARD_COMM_SPI_TX_DMA_HANDLER   DMA1_Channel5_IRQHandler
    #define BOARD_COMM_SPI_RX_DMA_IRQn      DMA1_Channel4_IRQn
    #define BOARD_COMM_SPI_TX_DMA_IRQn      DMA1_Channel5_IRQn


    #define BOARD_COMM_DATA_RDY_PORT        GPIOA
    #define BOARD_COMM_DATA_RDY_PIN_SRC     GPIO_PinSource0
    #define BOARD_COMM_DATA_RDY_PIN         GPIO_Pin_0

    #define BOOTLOADER_CHECK_PORT           GPIOA
    #define BOOTLOADER_CHECK_PIN_SRC        GPIO_PinSource1
    #define BOOTLOADER_CHECK_PIN            GPIO_Pin_1

    #define BOARD_COMM_EXTI_PORT            GPIOA
    #define BOARD_COMM_EXTI_PORT_SRC        EXTI_PortSourceGPIOA
    #define BOARD_COMM_EXTI_PIN_SRC         EXTI_PinSource1
    #define BOARD_COMM_EXTI_PIN             GPIO_Pin_1
    #define BOARD_COMM_EXTI_IRQn            EXTI1_IRQn
    #define BOARD_COMM_EXTI_HANDLER         EXTI1_IRQHandler
    #define BOARD_COMM_EXTI_LINE            EXTI_Line1

#endif