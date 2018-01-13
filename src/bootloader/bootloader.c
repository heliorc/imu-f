#include "includes.h"
#include "boothandler.h"
#include "bootloader_commands.h"
#include "hal_init.h"
#include "flash.h"
#include "report.h"

SPI_HandleTypeDef messageHandle;


volatile BootloaderCommandTypdef bootloaderCommand;

uint32_t report(uint8_t *txData, uint8_t *rxData, uint8_t length)
{
    spi_init(&messageHandle, SPI2, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, SPI2_IRQn, 1, 0);

   volatile HAL_SPI_StateTypeDef spiState = HAL_SPI_GetState(messageHandle);
   // ensure that SPI resources are available, but don't block if they are not
   if (spiState == HAL_SPI_STATE_READY) {
   
   report_info();
       HAL_SPI_TransmitReceive_IT(messageHandle, txData, rxData, length);

       return 1;
   } else {
       return 0;
   }
}

static void bootloader_main(void)
{
    while(1)
    {
        switch (bootloaderCommand)
        {
            case BL_ERASE_ALL:
            erase_flash(APP_ADDRESS, FLASH_END);
            break;
            case BL_ERASE_PAGE:
            erase_page();
            break;
            case BL_ERASE_ADDRESS_RANGE:
            erase_range();
            break;
            case BL_REPORT_INFO:

            report_info();
            break;
            case BL_BOOT_TO_APP:
            break;
            case BL_BOOT_TO_LOCATION:
            break;
            case BL_RESTART:
            break;
            case BL_WRITE_FIRMWARE:
            break;
            case BL_NONE:
            default:
            break;
        }
    }
}

void bootloader_start(void)
{
    bootloaderCommand = BL_NONE;
    hal_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN, 0); 
    HAL_Delay(500);
    if (HAL_GPIO_ReadPin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == (uint32_t)GPIO_PIN_RESET)
    {
        bootloader_main();
    }
    else 
    {
        BootToAddress(APP_ADDRESS);
    }
}
