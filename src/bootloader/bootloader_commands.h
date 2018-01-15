#pragma once
#include "includes.h"

typedef enum 
{
    BL_IDLE = 0,
    BL_ERASE_ALL = 1,
    // BL_ERASE_PAGE = 2,
    BL_ERASE_ADDRESS_RANGE = 3,
    BL_REPORT_INFO = 4,
    BL_BOOT_TO_APP = 5,
    BL_BOOT_TO_LOCATION = 6,
    BL_RESTART = 7,
    BL_WRITE_FIRMWARE = 8,
    BL_PREPARE_PROGRAM = 9,
    BL_END_PROGRAM = 10
} BootloaderCommandTypdef;

typedef struct bootloaderCommand {
   uint8_t version;
   BootloaderCommandTypdef command;
   uint32_t param1;
   uint32_t param2;
   uint32_t param3;
   uint32_t param4;
   uint32_t param5;
   uint32_t param6;
   uint32_t param7;
   uint32_t param8;
   BootloaderCommandTypdef crc;
} __attribute__ ((__packed__)) bootloaderCommand_t;