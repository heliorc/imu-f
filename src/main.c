#include "includes.h"
#include "gyro.h"
#include "board.h"
#include "board_comm.h"
#include "spi.h"
#include "adc.h"
#include "imu.h"

#ifdef C3PUBL
int main(void)
{
    //init board
    board_init();
    //start and go into BL code
    bootloader_start();
    while(1); //we never get here, but main isn't supposed to return on an MCU
}
#else
int main(void)
{
    //init board
    board_init();
    set_version();
    //init imu
    init_imu();
    //init gyro and its spi
    gyro_init();

    //setup board comm callback
    spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_callback_function;
    //init boad comm spi
    board_comm_init();

    // gyro_passthrough_start();
    while(1)
    {
        //what do we do in main loop, or do we keep everthing event based?

        //setup board comm timeout handler
        check_board_comm_setup_timeout();
        //low prioirty stuff in main loop. 
        update_quaternions(); //We have 8 gyro cycles to do this in and it should only take one
    }
}
#endif
