#include "includes.h"
#include "bootloader.h"

//here just so it can compuile
typedef enum 
{
    BC_NONE                 = 0,
    BL_ERASE_ALL            = 22,
    BL_ERASE_ADDRESS_RANGE  = 23,
    BL_REPORT_INFO          = 24,
    BL_BOOT_TO_APP          = 25,
    BL_BOOT_TO_LOCATION     = 26,
    BL_RESTART              = 27,
    BL_WRITE_FIRMWARE       = 28,
    BL_WRITE_FIRMWARES      = 29,
    BL_PREPARE_PROGRAM      = 30,
    BL_END_PROGRAM          = 31,
    BC_IMUF_CALIBRATE       = 99,
    BC_IMUF_LISTENING       = 108,
    BC_IMUF_REPORT_INFO     = 121,
    BC_IMUF_SETUP           = 122,
    BC_IMUF_RESTART         = 127,
} imufCommandsList_t;

typedef struct imufCommand {
   uint32_t param0;
   uint32_t command;
   uint32_t param1;
   uint32_t param2;
   uint32_t param3;
   uint32_t param4;
   uint32_t param5;
   uint32_t param6;
   uint32_t param7;
   uint32_t param8;
   uint32_t param9;
   uint32_t param10;
   uint32_t crc;
   uint32_t param11;
} __attribute__ ((aligned (16), packed)) imufCommand_t;


static void run_command(volatile imufCommand_t *command)
{
    switch (command->command)
    {
        case BL_ERASE_ALL:
            erase_flash(APP_ADDRESS, FLASH_END);
        break;
        case BL_ERASE_ADDRESS_RANGE:
            erase_range(command->param1, command->param2);
        break;
        case BL_REPORT_INFO:
        //memset(boardCommSpixxBuffer, 0, COM_BUFFER_SIZE);       
        //get_report_info(&boardCommSPIHandle, boardCommSpixxBuffer, boardCommSpixxBuffer);
        break;
        case BL_BOOT_TO_APP:
            boot_to_address(APP_ADDRESS);
        break;
        case BL_BOOT_TO_LOCATION:
            boot_to_address(command->param1);
        break;
        case BL_RESTART:
            boot_to_address(THIS_ADDRESS);
        break;
        case BL_WRITE_FIRMWARE:
            flash_program_word(command->param1, command->param2);
        break;
        case BL_WRITE_FIRMWARES:
            //write 8 words in one spi transaction
            flash_program_word(command->param1, command->param2);
            flash_program_word(command->param1, command->param3);
            flash_program_word(command->param1, command->param4);
            flash_program_word(command->param1, command->param5);
            flash_program_word(command->param1, command->param6);
            flash_program_word(command->param1, command->param7);
            flash_program_word(command->param1, command->param8);
            flash_program_word(command->param1, command->param9);
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
    single_gpio_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN_SRC, BOOTLOADER_CHECK_PIN, 0, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP);
    //todo: add delay functions

    //If boothandler tells us to, or if pin is hi, we enter BL mode
    if ( (BOOT_MAGIC_ADDRESS == THIS_ADDRESS) || read_digital_input(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) )
    {
        //register callback for transfer complete
        //spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = bootloader_spi_callback;
        //init board comm SPI
        //board_comm_init();
        //that's it, everything else is event based
        while(1);
    }
    else 
    {
        //boot to app
        boot_to_address(APP_ADDRESS);
    }
}