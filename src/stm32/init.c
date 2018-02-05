#include "includes.h"

void board_init(void)
{
    SystemInit();

    SCB->VTOR = THIS_ADDRESS; //set vector register to firmware start
	__enable_irq();           // enable interrupts

    SystemCoreClockUpdate();
    clock_config(); // lives in clock.c
}
