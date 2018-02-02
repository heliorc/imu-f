#include "includes.h"

void board_init(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    clock_config(); // lives in clock.c
}
