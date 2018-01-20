#pragma once
#include "includes.h"

typedef enum 
{
    BC_NONE = 0,
    BC_GYRO_SETUP = 1,
} BoardCommandTypdef;


typedef struct boardCommand {
   uint8_t version;
   BoardCommandTypdef command;
   uint32_t param1;
   uint32_t param2;
   uint32_t param3;
   uint32_t param4;
   uint32_t param5;
   uint32_t param6;
   uint32_t param7;
   uint32_t param8;
   BoardCommandTypdef crc;
} __attribute__ ((__packed__)) boardCommand_t;

extern void board_comm_init(void);
extern void board_comm_callback_function(SPI_HandleTypeDef *hspi);