#pragma once
#include "includes.h"

extern void bootloader_start(void);
extern void bootloader_spi_callback(SPI_HandleTypeDef *hspi);