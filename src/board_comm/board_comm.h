#pragma once
#include "includes.h"

#define COM_BUFFER_SIZE 48

typedef enum gyroToBoardCommMode
{
    GTBCM_SETUP                  = 41,
    GTBCM_GYRO_ONLY_PASSTHRU     = 7,
    GTBCM_GYRO_ACC_PASSTHRU      = 16,
    GTBCM_GYRO_ONLY_FILTER_F     = 17,
    GTBCM_GYRO_ACC_FILTER_F      = 29,
    GTBCM_GYRO_ACC_QUAT_FILTER_F = 45,
} gyroToBoardCommMode_t;

typedef struct boardCommState {
   uint32_t commMode;
   uint32_t bufferSize;
} boardCommState_t;

typedef enum 
{
    BC_NONE                 = 0,
    BL_ERASE_ALL            = 22,
    BL_ERASE_ADDRESS_RANGE  = 23,
    BL_REPORT_INFO          = 24,
    BL_BOOT_TO_APP          = 25,
    BL_BOOT_TO_LOCATION     = 26,
    BL_RESTART              = 27,
    BL_WRITE_FIRMWARE       = 28,
    BL_WRITE_FIRMWARES      = 29,
    BL_PREPARE_PROGRAM      = 30,
    BL_END_PROGRAM          = 31,
    BC_IMUF_CALIBRATE       = 99,
    BC_IMUF_REPORT_INFO     = 121,
    BC_IMUF_SETUP           = 122,
    BC_IMUF_RESTART         = 127,
} imufCommandsList_t;


typedef struct imufCommand {
   uint32_t command;
   uint32_t param1;
   uint32_t param2;
   uint32_t param3;
   uint32_t param4;
   uint32_t param5;
   uint32_t param6;
   uint32_t param7;
   uint32_t param8;
   uint32_t crc;
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
extern void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);