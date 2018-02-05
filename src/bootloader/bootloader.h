#pragma once
#include "includes.h"

#define BOOTLOADER_VERSION 101

extern void bootloader_start(void);
extern void bootloader_spi_callback_function(void);