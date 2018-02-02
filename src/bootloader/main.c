#include "includes.h"
#include "bootloader.h"


int main(void)
{
    board_init();       //inits the clocks  
    bootloader_start(); //init the bootloader,
    while(1);
}