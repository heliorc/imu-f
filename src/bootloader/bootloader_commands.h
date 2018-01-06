#pragma once
#include "includes.h"

typedef enum 
{
    BL_NONE = 0,
    BL_ERASE_ALL = 1,
    BL_ERASE_PAGE = 2,
    BL_ERASE_ADDRESS = 3,
    BL_REPORT_INFO = 4,
    BL_BOOT_TO_APP = 5,
    BL_BOOT_TO_LOCATION = 6,
    BL_RESTART = 7,
    BL_WRITE_FIRMWARE = 8
}  BootloaderCommandTypdef;

typedef struct VersionInfoTypedef
{   
    uint32_t hardware;
    uint32_t firmware;
    uint32_t bootloader;
    uint32_t uid1;
    uint32_t uid2;
    uint32_t uid3;
} VersionInfoTypedef_t;