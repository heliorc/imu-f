#pragma once
//springboard header file
#include "includes.h"

#define MOTOLAB

#if defined(OMNIBUS)
    #include "omnibus.h"
#elif defined(MOTOLAB)
    #include "motolab.h"
#else
    //chip select pin
    #define GYRO_CS_PIN       GPIO_PIN_9
    //chip select port
    #define GYRO_CS_GPIO      GPIOA
    #define GYRO_SPI          SPI2

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
#endif