#pragma once
#include "bootloader_commands.h"

typedef struct VersionInfoTypedef
{   
    uint32_t hardware;
    uint32_t firmware;
    uint32_t bootloader;
    uint32_t uid1;
    uint32_t uid2;
    uint32_t uid3;
} VersionInfoTypedef_t;

extern void get_report_info(SPI_HandleTypeDef* messageHandle, char* rxData, char* txData);