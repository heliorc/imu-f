#include "includes.h"
#include "bootloader.h"


int main(void)
{
    board_init();       //inits the clocks  
    set_version();      //fill version info (should be static instead)
    bootloader_start(); //init the bootloader,
    while(1);
}