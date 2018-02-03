#pragma once
#include "includes.h"
extern volatile uint32_t debug1;
extern volatile uint32_t debug2;

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
} __attribute__((__packed__)) filteredData_t;


#define GYRO_BUFFER_SIZE 256

#define CALIBRATION_CYCLES 1900

extern volatile int calibratingGyro;
extern volatile axisData_t gyroSum;
extern volatile axisData_t gyroCalibrationTrim;

extern int skipGyro;

extern void gyro_init(void);
extern void gyro_exti_callback(void);