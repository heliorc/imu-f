/*
*****************************************************************************
**
**  File        : stm32_flash.ld
**
**  Abstract    : Linker script for STM32F30x Device with
**                128KByte FLASH and 40KByte RAM
**
*****************************************************************************
*/

/* Specify the memory areas. */
MEMORY
{
  FLASH  (rx)     : ORIGIN = 0x08001800, LENGTH = 26K /* last 2kb used for config storage */
  RAM    (xrw)    : ORIGIN = 0x20000000, LENGTH = 16K
  MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
}

INCLUDE "stm32_flash.ld"