#pragma once
#include "includes.h"

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
    axisData_t rateData;
    axisData_t accData;
    float tempC;
    float quaternion[4];
    uint32_t crc;
} __attribute__((__packed__)) filteredData_t;

typedef struct gyro_settings_config {  //unpacked, aligned 4
    uint32_t orientation;
    int32_t smallX;
    int32_t smallY;
    int32_t smallZ;
} __attribute__((__packed__)) gyro_settings_config_t;

extern volatile gyro_settings_config_t gyroSettingsConfig;

extern volatile int calibratingGyro;
	 
//called when new settings are sent
extern void reset_matrix(void);
extern void gyro_init(void); 	 extern void gyro_init(void);