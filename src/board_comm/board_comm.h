#pragma once
#include "includes.h"

#define COM_BUFFER_SIZE 48
#define DEFAULT_COM_SIZE 32

typedef enum gyroToBoardCommMode
{
    GTBCM_SETUP                  = 32,
    GTBCM_GYRO_ONLY_PASSTHRU     = 6,
    GTBCM_GYRO_ACC_PASSTHRU      = 15,
    GTBCM_GYRO_ONLY_FILTER_F     = 12,
    GTBCM_GYRO_ACC_FILTER_F      = 28,
    GTBCM_GYRO_ACC_QUAT_FILTER_F = 44,
} gyroToBoardCommMode_t;

typedef struct boardCommState {
   gyroToBoardCommMode_t commMode;
   int commEnabled;
   uint32_t bufferSize;
} boardCommState_t;

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
extern uint8_t boardCommSpiRxBuffer[COM_BUFFER_SIZE];
extern uint8_t boardCommSpiTxBuffer[COM_BUFFER_SIZE];

extern volatile boardCommState_t boardCommState;

extern void board_comm_init(void);
extern void board_comm_callback_function(SPI_HandleTypeDef *hspi);
extern void parse_imuf_command(imufCommand_t* newCommand, uint8_t* buffer);
