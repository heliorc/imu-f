#include "includes.h"
#include "boothandler.h"
#include "bootloader_commands.h"
#include "hal_init.h"
#include "flash.h"
#include "report.h"


char txBuffer[256];
char rxBuffer[256];
SPI_HandleTypeDef MESSAGE_HANDLE;

volatile bootloaderCommand_t newCommand;
volatile uint32_t flush;

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
        memset(txBuffer, 0, 256);       
        get_report_info(MESSAGE_HANDLE, txBuffer, rxBuffer);
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
    bootloaderCommand = BL_NONE;
    hal_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN, 0); 
    HAL_Delay(500);
    if (HAL_GPIO_ReadPin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == (uint32_t)GPIO_PIN_RESET)
    {
        spi_init(&MESSAGE_HANDLE, SPI2, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, SPI2_IRQn, 1, 0);
        HAL_SPI_TransmitReceive_IT(&MESSAGE_HANDLE, txData, rxData, 256);
        while(1){

        }
    }
    else 
    {
        BootToAddress(APP_ADDRESS);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == SPI2)
    {
        //parse! hspi->
        memccpy(&newCommand, rxBuffer, sizeOf(bootloaderCommand_t));
        if (newCommand.command && newCommand.command == newCommand.crc){
            run_command(&newCommand);
        }
        memset(rxBuffer, 0, 256);
        HAL_SPI_TransmitReceive_IT(&MESSAGE_HANDLE, txData, rxData, 256);
    }
}