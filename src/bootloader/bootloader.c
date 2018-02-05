#include "includes.h"
#include "boothandler.h"
#include "board_comm.h"
#include "hal_gpio_init.h"
#include "spi.h"
#include "flash.h"
#include "report.h"


static void run_command(volatile imufCommand_t *command)
{
    switch (command->command)
    {
        case BL_ERASE_ALL:
        erase_flash(APP_ADDRESS, FLASH_END);
        break;
        // case BL_ERASE_PAGE:
        // erase_page();
        // break;
        case BL_ERASE_ADDRESS_RANGE:
        erase_range(command->param1, command->param2);
        break;
        case BL_REPORT_INFO:
        //memset(boardCommSpixxBuffer, 0, COM_BUFFER_SIZE);       
        //get_report_info(&boardCommSPIHandle, boardCommSpixxBuffer, boardCommSpixxBuffer);
        break;
        case BL_BOOT_TO_APP:
        BootToAddress(APP_ADDRESS);
        break;
        case BL_BOOT_TO_LOCATION:
        BootToAddress(command->param1);
        break;
        case BL_RESTART:
        BootToAddress(THIS_ADDRESS);
        break;
        case BL_WRITE_FIRMWARE:
        flash_program_word(command->param1, command->param2);
        break;
        case BL_WRITE_FIRMWARES:
        flash_program_word(command->param1, command->param2);
        flash_program_word(command->param1, command->param3);
        flash_program_word(command->param1, command->param4);
        flash_program_word(command->param1, command->param5);
        flash_program_word(command->param1, command->param6);
        flash_program_word(command->param1, command->param7);
        flash_program_word(command->param1, command->param8);
        break;
        case BL_PREPARE_PROGRAM:
        prepare_flash_for_program();
        break;
        case BL_END_PROGRAM:
        end_flash_for_program();
        break;
        default:
        break;
    }
}

void bootloader_start(void)
{
    //setup bootloader pin then wait 50 ms
    hal_gpio_init_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, GPIO_MODE_INPUT, GPIO_PULLUP, 0); 
    HAL_Delay(50);
    //If boothandler tells us to, or if pin is hi, we enter BL mode
    if ( (BOOT_MAGIC_ADDRESS == THIS_ADDRESS) || HAL_GPIO_ReadPin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == (uint32_t)GPIO_PIN_RESET )
    {
        //register callback for transfer complete
        spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = bootloader_spi_callback;
        //init board comm SPI
        board_comm_init();
        //that's it, everything else is event based
        while(1);
    }
    else 
    {
        //boot to app
        BootToAddress(APP_ADDRESS);
    }
}

void bootloader_spi_callback(SPI_HandleTypeDef *hspi)
{

    //rx complete, do pin thing here
    HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
    timeBoardCommSetupIsr = HAL_GetTick();

    if (parse_imuf_command(&imufCommandRx))
    {
        run_command(&imufCommandRx);  //this command will handle the message start handling
    }

    //restart listening if in setup mode
    timeBoardCommSetupIsr = 0;

}