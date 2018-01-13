#pragma once
//springboard header file
#include "includes.h"

#define RCC_ALL_CLK (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2)

#if defined(OMNIBUS)
    #include "omnibus.h"
#elif defined(MOTOLAB)
    #include "motolab.h"
#else
    #define GYRO_SPI            SPI3
    #define GYRO_SPI_IRQn       SPI3_IRQn
    #define GYRO_CS_HARDWARE    0
    #define GYRO_CS_PIN         GPIO_PIN_12
    #define GYRO_CS_PORT        GPIOB
    #define GYRO_CS_ALTERNATE   GPIO_AF5_SPI2
    #define GYRO_MISO_PIN       GPIO_PIN_14
    #define GYRO_MISO_PORT      GPIOB
    #define GYRO_MISO_ALTERNATE GPIO_AF5_SPI2
    #define GYRO_MOSI_PIN       GPIO_PIN_15
    #define GYRO_MOSI_PORT      GPIOB
    #define GYRO_MOSI_ALTERNATE GPIO_AF5_SPI2
    #define GYRO_SCK_PIN        GPIO_PIN_13
    #define GYRO_SCK_PORT       GPIOB
    #define GYRO_SCK_ALTERNATE  GPIO_AF5_SPI2
    #define SPI2_ENABLED      1
    #define SPI2_HARDWARE_CS  0
    #define SPI2_CS_PIN       0
    #define SPI2_CS_PORT      0
    #define SPI2_MISO_PIN     GPIO_PIN_10
    #define SPI2_MISO_PORT    GPIOA
    #define SPI2_MOSI_PIN     GPIO_PIN_11
    #define SPI2_MOSI_PORT    GPIOA
    #define SPI2_SCK_PIN      GPIO_PIN_1
    #define SPI2_SCL_PORT     GPIOF


    #define BOOTLOADER_CHECK_PORT  GPIOB
    #define BOOTLOADER_CHECK_PIN   GPIO_PIN_5
#endif