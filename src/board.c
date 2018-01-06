#include "stm32/stm32_init.h"
#include "gpio/gpio_init.h"
#include "gyro/gyro_init.h"

void board_init(void)
{
    stm32_init();
    gpio_init();
}
