#pragma once
#include "includes.h"

extern SPI_HandleTypeDef gyroSpiHandle;
extern SPI_HandleTypeDef outSpiHandle;
extern void spi_init(uint32_t baudRatePrescaler);