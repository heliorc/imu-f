#pragma once
#include "includes.h"

enum
{
    UNKNOWN_ERROR = 1 << 0,
    GYRO_SETUP_COMMUNICATION_FAILIURE = 1 << 1,
};

extern void error_handler(uint32_t error);