#pragma once
#include "bootloader_commands.h"

extern void get_report_info(VersionInfoTypedef* info);


typedef struct VersionInfoTypedef
{   
    uint32_t hardware;
    uint32_t firmware;
    uint32_t bootloader;
    uint32_t uid1;
    uint32_t uid2;
    uint32_t uid3;
} VersionInfoTypedef_t;