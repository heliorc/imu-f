#include "includes.h"
#include "bootloader.h"
#include "board_comm.h"
#include "config.h"
#include "caesar.h"


static void run_command(volatile imufCommand_t *command, volatile imufCommand_t *reply)
{
    clear_imuf_command(reply);
    switch (command->command)
    {
        case BL_ERASE_ALL:
            erase_flash(APP_ADDRESS, FLASH_END);
            reply->command = reply->crc = BL_ERASE_ALL;
        break;
        case BL_ERASE_ADDRESS_RANGE:
            erase_range(command->param1, command->param2);
            reply->command = reply->crc = BL_ERASE_ADDRESS_RANGE;
        break;
        case BL_REPORT_INFO:
            memcpy((uint8_t*)&(reply->param1), (uint8_t*)&flightVerson, sizeof(flightVerson));
            reply->command = reply->crc = BL_REPORT_INFO;
        break;
        case BL_BOOT_TO_APP:
            boot_to_address(THIS_ADDRESS);    //can't reply of course
        break;
        case BL_BOOT_TO_LOCATION:
            //boot_to_address(command->param1); //can't reply of course
            boot_to_address(THIS_ADDRESS);    //can't reply of course
        break;
        case BL_RESTART:
            boot_to_address(THIS_ADDRESS);    //can't reply of course
        break;
        case BL_WRITE_FIRMWARE:
            flash_program_word(command->param1, command->param2);
            reply->command = reply->crc = BL_WRITE_FIRMWARE;
        break;
        case BL_WRITE_FIRMWARES:
            //write 8 words in one spi transaction
            flash_program_word(command->param1, command->param2);
            flash_program_word(command->param1+4, command->param3);
            flash_program_word(command->param1+8, command->param4);
            flash_program_word(command->param1+12, command->param5);
            flash_program_word(command->param1+16, command->param6);
            flash_program_word(command->param1+20, command->param7);
            flash_program_word(command->param1+24, command->param8);
            flash_program_word(command->param1+28, command->param9);
            reply->command = reply->crc = BL_WRITE_FIRMWARES;
        break;
        case BL_PREPARE_PROGRAM:
            prepare_flash_for_program();
            reply->command = reply->crc = BL_PREPARE_PROGRAM;
        break;
        case BL_END_PROGRAM:
            end_flash_for_program();
            reply->command = reply->crc = BL_END_PROGRAM;
        break;
        default:
            //invalid command, just listen now
            bcTx.command = bcTx.crc = BL_LISTENING;
        break;
    }
}

void bootloader_start(void)
{
    //delay_ms(10);
    //boot_to_address(APP_ADDRESS);
    //setup bootloader pin then wait 30 ms

    single_gpio_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN_SRC, BOOTLOADER_CHECK_PIN, 0, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN);
    if(complex_boot())
    {
        single_gpio_init(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN_SRC, BOARD_COMM_DATA_RDY_PIN, 0, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_DOWN);
        delay_ms(30);
    }

    //If boothandler tells us to, or if pin is hi, we enter BL mode
    if ( (BOOT_MAGIC_ADDRESS == THIS_ADDRESS) || read_digital_input(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN) )
    //if ( 1 ) //testing, force bl mode
    {

        //set callback function
        spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = bootloader_spi_callback_function;
        //init board comm spi
        if(complex_boot())
        {
            board_comm_init();
        }
        //clear imuf commands
        clear_imuf_command(&bcRx);
        clear_imuf_command(&bcTx);
        //set first command, which is to listen
        bcTx.command = bcTx.crc = BL_LISTENING;
        //start the process
        if(complex_boot())
        {
            start_listening();
        }
        //everything else is event based
        while(1)
        {
        }
    }
    else 
    {
        //boot to app
        if(complex_boot())
        {
            boot_to_address(APP_ADDRESS);
        }
    }
}

void bootloader_spi_callback_function(void)
{
    board_comm_spi_complete(); //this needs to be called when the transaction is complete

    if ( (bcTx.command == BL_LISTENING) && parse_imuf_command(&bcRx) )//we  were waiting for a command //we have a valid command
    {
        //command checks out
        //run the command and generate the reply
        run_command(&bcRx,&bcTx); 
        
    }
    else
    {
        //bad command, listen for another
        clear_imuf_command(&bcRx);
        clear_imuf_command(&bcTx);
        bcTx.command = bcTx.crc = BL_LISTENING;
    }
    //start the process again, bootloader always does this, no need to jump into runtime for gyro stuff
    start_listening();
}