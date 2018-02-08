#include "includes.h"
#include "error_handler.h"

void error_handler(errorMask_t error)
{
    (void)(error);
    while(1);
}

void super_error(uint32_t time)
{
    volatile int cat = 1;
    single_gpio_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN_SRC, BOOTLOADER_CHECK_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 0);
    while(1)
    {
        cat++;
        gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);
        delay_ms(time);
        gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 0);
        delay_ms(time);
    }
}

void pulse_error(uint32_t time)
{
    volatile int cat = 1;
    single_gpio_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN_SRC, BOOTLOADER_CHECK_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 0);

    gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);
    delay_ms(time);
    gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 0);
    delay_ms(time);

}
