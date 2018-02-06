#pragma once
#include "includes.h"

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

extern volatile int calibratingGyro;
extern volatile filteredData_t filteredData;

extern void gyro_init(void);