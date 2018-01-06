#include "includes.h"
#include "gyro/passthrough_start.h"
#include "board.h"

#ifdef C3PUBL
int main(void)
{
    bootloader_start();
    return(0);
}
#else
int main(void)
{
    board_init();
    gyro_passthrough_start();
    return(0);
}
#endif
