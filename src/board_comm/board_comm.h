#pragma once
#include "includes.h"

#define COM_BUFFER_SIZE 256

typedef enum 
{
    NONE = 0,
    BC_GYRO_SETUP = 1,
    BL_ERASE_ALL = 2,
    // BL_ERASE_PAGE = 2,
    BL_ERASE_ADDRESS_RANGE = 3,
    BL_REPORT_INFO = 4,
    BL_BOOT_TO_APP = 5,
    BL_BOOT_TO_LOCATION = 6,
    BL_RESTART = 7,
    BL_WRITE_FIRMWARE = 8,
    BL_PREPARE_PROGRAM = 9,
    BL_END_PROGRAM = 10   
} imufCommandsList_t;


typedef struct imufCommand {
   uint8_t version;
   imufCommandsList_t command;
   uint32_t param1;
   uint32_t param2;
   uint32_t param3;
   uint32_t param4;
   uint32_t param5;
   uint32_t param6;
   uint32_t param7;
   uint32_t param8;
   imufCommandsList_t crc;
} __attribute__ ((__packed__)) imufCommand_t;

extern SPI_HandleTypeDef boardCommSPIHandle;
extern DMA_HandleTypeDef hdmaBoardCommSPIRx;
extern DMA_HandleTypeDef hdmaBoardCommSPITx;
uint8_t boardCommSpiRxBuffer[COM_BUFFER_SIZE];
uint8_t boardCommSpiTxBuffer[COM_BUFFER_SIZE];

extern void board_comm_init(void);
extern void board_comm_callback_function(SPI_HandleTypeDef *hspi);
extern void parse_imuf_command(imufCommand_t* newCommand, uint8_t* buffer);
