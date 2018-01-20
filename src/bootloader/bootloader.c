#include "includes.h"
#include "boothandler.h"
#include "bootloader_commands.h"
#include "hal_gpio_init.h"
#include "spi.h"
#include "board_comm.h"
#include "flash.h"
#include "report.h"

bootloaderCommand_t newCommand;

void run_command(bootloaderCommand_t* bl_command)
{
    switch (bl_command->command)
    {
        case BL_ERASE_ALL:
        erase_flash(APP_ADDRESS, FLASH_END);
        break;
        // case BL_ERASE_PAGE:
        // erase_page();
        // break;
        case BL_ERASE_ADDRESS_RANGE:
        erase_range(bl_command->param1, bl_command->param2);
        break;
        case BL_REPORT_INFO:
        memset(boardCommSpiTxBuffer, 0, 256);       
        get_report_info(&boardCommSPIHandle, boardCommSpiRxBuffer, boardCommSpiTxBuffer);
        break;
        case BL_BOOT_TO_APP:
        BootToAddress(APP_ADDRESS);
        break;
        case BL_BOOT_TO_LOCATION:
        BootToAddress(bl_command->param1);
        break;
        case BL_RESTART:
        BootToAddress(THIS_ADDRESS);
        break;
        case BL_WRITE_FIRMWARE:
        flash_program_word(bl_command->param1, bl_command->param2);
        break;
        case BL_PREPARE_PROGRAM:
        prepare_flash_for_program();
        break;
        case BL_END_PROGRAM:
        end_flash_for_program();
        break;
        case BL_NONE:
        default:
        break;
    }
}

void bootloader_start(void)
{
    newCommand.command = BL_NONE;
    memset(boardCommSpiRxBuffer, 0, 256);   
    memset(boardCommSpiTxBuffer, 0, 256);   
    hal_gpio_init_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN, 0); 
    HAL_Delay(500);
    if ( HAL_GPIO_ReadPin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == (uint32_t)GPIO_PIN_RESET )
    {
        spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = bootloader_spi_callback;
        board_comm_init();
        while(1){

        }
    }
    else 
    {
        BootToAddress(APP_ADDRESS);
    }
}

void bootloader_spi_callback(SPI_HandleTypeDef *hspi)
{
    memcpy(&newCommand, boardCommSpiRxBuffer, sizeof(bootloaderCommand_t));
    if (newCommand.command && newCommand.command == newCommand.crc){
        run_command(&newCommand);
    }
    memset(boardCommSpiRxBuffer, 0, 256);
    //setup for next DMA transfer
    HAL_SPI_TransmitReceive_IT(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, 256);
}