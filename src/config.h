#pragma once
#include "includes.h"

//for F3's only
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

typedef struct flightVersionInfoTypedef
{   
    uint32_t hardware;
    uint32_t firmware;
    uint32_t bootloader;
    uint32_t uid1;
    uint32_t uid2;
    uint32_t uid3;
} flightVersionInfoTypedef_t;

extern flightVersionInfoTypedef_t flightVerson;

extern void set_version(void);