#pragma once
#include "includes.h"
#include "gyro.h"

typedef enum filterAxisTypedef
{
    PITCH = 0,
    YAW = 1,
    ROLL = 2
} filterAxisTypedef_t;

typedef struct fastKalman {
    float q;       //process noise covariance
    float r;       //measurement noise covariance
    float p;       //estimation error covariance matrix
    float k;       //kalman gain
    float x;       //state
    float lastX;   //previous state
    int gyroDfkfDataPtr;
    float gyroDfkfData[32];
} fastKalman_t;

typedef enum filterTypedef
{
    NO_ESTIMATION = 0,
    STD_DEV_ESTIMATION = 1,
    DISTANCE_ESTIMATION = 2
} filterTypedef_t;

extern void fastKalmanInit(float q, float r, float p, float intialValue, filterTypedef_t type);
extern float fastKalmanUpdate(filterAxisTypedef_t axis, float input);
extern void filter_data(axisData_t* gyroRateData,axisData_t* gyroAccData,float gyroTempData, filteredData_t* filteredData);