#pragma once
#include "includes.h"

//error is a bitmask
typedef enum errorMask
{
    UNKNOWN_ERROR = 1 << 0,
    GYRO_SETUP_COMMUNICATION_FAILIURE = 1 << 1,
    SPI_INIT_FAILIURE = 1 << 2,
    SPI_RX_DMA_INIT_FAILIURE = 1 << 3,
    SPI_TX_DMA_INIT_FAILIURE = 1 << 4,
    GYRO_DETECT_FAILURE = 1 << 5,
} errorMask_t;

extern void error_handler(errorMask_t error);