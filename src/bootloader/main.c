#include "includes.h"
#include "bootloader.h"
#include "crc.h"


int main(void)
{
    board_init();       //inits the clocks  
    set_version();      //fill version info (should be static instead)
    crc_config();       //start the crc hardware
    bootloader_start(); //init the bootloader,
    while(1);
}