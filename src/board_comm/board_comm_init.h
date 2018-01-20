#pragma once
#include "includes.h"

extern SPI_HandleTypeDef boardCommSPIHandle;

extern DMA_HandleTypeDef hdmaBoardCommSPIRx;
extern DMA_HandleTypeDef hdmaBoardCommSPITx;

extern void board_comm_init(void);
