#include "includes.h"
#include "gyro.h"
#include "board.h"
#include "board_comm.h"
#include "spi.h"

#ifdef C3PUBL
int main(void)
{
    board_init();
    bootloader_start();
    return(0);
}
#else
int main(void)
{
    board_init();
    spiCallbackFunctionArray[BOARD_COMM_SPI_NUM] = board_comm_callback_function;
    board_comm_init();
    gyro_init();
    gyro_passthrough_start();
    return(0);
}
#endif
