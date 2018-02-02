#pragma once
#include "includes.h"

typedef enum gyroToBoardCommMode
{
    GTBCM_SETUP                  = 52, //max number
    GTBCM_GYRO_ONLY_PASSTHRU     = 6,  //no crc, gyro, 3*2 bytes
    GTBCM_GYRO_ACC_PASSTHRU      = 14, //no crc, acc, temp, gyro, 3*2, 1*2, 3*2 bytes
    GTBCM_GYRO_ONLY_FILTER_F     = 16, //gyro, filtered, 3*4 bytes, 4 bytes crc
    GTBCM_GYRO_ACC_FILTER_F      = 28, //gyro, filtered, acc, 3*4, 3*4, 4 byte crc
    GTBCM_GYRO_ACC_QUAT_FILTER_F = 48, //gyro, filtered, temp, filtered, acc, quaternions, filtered, 3*4, 3*4, 4*4, 1*4, 4 byte crc
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
    BL_LISTENING            = 32,
    BC_IMUF_CALIBRATE       = 99,
    BC_IMUF_LISTENING       = 108,
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
   uint32_t param9;
   uint32_t param10;
   uint32_t crc;
   uint32_t syncWord; //overflow, used for sync
} __attribute__ ((packed)) imufCommand_t;

extern volatile imufCommand_t bcRx;
extern volatile imufCommand_t bcTx;
extern volatile uint32_t spiDoneFlag;

extern void clear_imuf_command(volatile imufCommand_t* command);
extern void volatile_uint32_copy(volatile uint32_t* dst, volatile uint32_t* src, uint32_t size);
extern void board_comm_init(void);
extern int  parse_imuf_command(volatile imufCommand_t* command);
extern void start_listening(void);
extern void board_comm_spi_complete(void);