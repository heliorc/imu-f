#include "includes.h"
#include "gyro.h"
#include "board.h"
#include "board_comm.h"
#include "spi.h"

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
    //setup board comm callback
    spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_callback_function;
    //init boad comm spi
    board_comm_init();
    //init gyro and its spi
    gyro_init();
    //not used, replace with flight code init maybe
    gyro_passthrough_start();
    while(1)
    {
        //what do we do in main loop, or de we keep everthing event based?
    }
}
#endif
