#pragma once
#include "includes.h"

extern SPI_HandleTypeDef gyroSPIHandle;

extern DMA_HandleTypeDef hdmaGyroSPIRx;
extern DMA_HandleTypeDef hdmaGyroSPITx;

extern void gyro_init(void);