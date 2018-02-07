#include "includes.h"
#include "board_comm.h"
#include "quaternions.h"
#include "imu.h"
#include "gyro.h"

int main(void)
{
    //inits the clocks
    board_init();
    //this makes the status light go red
    single_gpio_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN_SRC, BOOTLOADER_CHECK_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    //init gyro
    gyro_init();

    ///////////////////////////////////////////////////////////////////
    ////board comm init, maybe all this should go into board_comm.c////
    ///////////////////////////////////////////////////////////////////
    spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_spi_callback_function; //set callback function
    board_comm_init();                           //init board comm spi
    clear_imuf_command(&bcRx);                   //clear imuf commands
    clear_imuf_command(&bcTx);                   //clear imuf commands
    bcTx.command = bcTx.crc = BC_IMUF_LISTENING; //set first command, which is to listen
    start_listening();                           //start the board comm process
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    //quaternions stuff happens in the main loop here
    while(1)
    {
        update_quaternions();
    }
}