/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F051K6 Device with
**                32KByte FLASH, 8KByte RAM
**
*****************************************************************************
*/

/*
0x8000000 RFBL
0x8003000 FIRMWARE
0x8007C00 Config
*/

/* Specify the memory areas. */
MEMORY
{
	FLASH (rx)      : ORIGIN = 0x08003000, LENGTH = 19K
	RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 8K
	MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"
