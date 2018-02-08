#pragma once
#include "includes.h"

#define SYSTICK_ISR_PRE_PRI 0
#define SYSTICK_ISR_SUB_PRI 0

//not using any of these
//#define BOARD_COMM_SPI_ISR_PRE_PRI 4
//#define BOARD_COMM_SPI_ISR_SUB_PRI 4
//#define BOARD_COMM_SPI_DMA_RX_PRE_PRI 4
//#define BOARD_COMM_SPI_DMA_RX_SUB_PRI 4
//#define BOARD_COMM_SPI_DMA_TX_PRE_PRI 4
//#define BOARD_COMM_SPI_DMA_TX_SUB_PRI 4

//exti to handle board comm cleanup
#define BOARD_COMM_EXTI_ISR_PRE_PRI 6
#define BOARD_COMM_EXTI_ISR_SUB_PRI 6

//#define GYRO_SPI_ISR_PRE_PRI 3
//#define GYRO_SPI_ISR_SUB_PRI 3
#define GYRO_SPI_DMA_RX_PRE_PRI 2
#define GYRO_SPI_DMA_RX_SUB_PRI 2
//#define GYRO_SPI_DMA_TX_PRE_PRI 2
//#define GYRO_SPI_DMA_TX_SUB_PRI 2

#define GYRO_EXTI_ISR_PRE_PRI 5
#define GYRO_EXTI_ISR_SUB_PRI 5