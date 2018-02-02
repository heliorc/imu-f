#include "includes.h"

void reset_mcu(void)
{
    NVIC_SystemReset();
}