#pragma once
#include "includes.h"
#include "gyro_device.h"

typedef struct axisDataInt
{
    int x;
    int y;
    int z;
} __attribute__((__packed__)) axisDataInt_t;

typedef struct axisData
{
    float x;
    float y;
    float z;
} __attribute__((__packed__)) axisData_t;

typedef struct filteredData
{
    axisData_t rateData; //3
    axisData_t accData;  //3
    float tempC;         //1
    float quaternion[4]; //4
    uint32_t crc;        //1
    uint32_t tail;       //1
} __attribute__((__packed__)) filteredData_t;

typedef struct gyro_settings_config {  //unpacked, aligned 4
    uint32_t orientation;
    int32_t smallX;
    int32_t smallY;
    int32_t smallZ;
} __attribute__((__packed__)) gyro_settings_config_t;

extern volatile gyro_settings_config_t gyroSettingsConfig;

extern volatile int calibratingGyro;
extern volatile int gyroDataReadDone;
	 
//called when new settings are sent
extern void reset_matrix(void);
extern void gyro_init(void);
extern void gyro_int_to_float(gyroFrame_t* gyroRxFrame);
extern void run_gyro_filters(void);
extern void increment_acc_tracker(void);
extern void fire_spi_send_ready();