#pragma once
#include "includes.h"

#define SYSTICK_ISR_PRE_PRI 0
#define SYSTICK_ISR_SUB_PRI 0

#define BOARD_COMM_SPI_ISR_PRE_PRI 1
#define BOARD_COMM_SPI_ISR_SUB_PRI 1

#define GYRO_SPI_ISR_PRE_PRI 1
#define GYRO_SPI_ISR_SUB_PRI 2

#define GYRO_EXTI_ISR_PRE_PRI 2
#define GYRO_EXTI_ISR_SUB_PRI 1