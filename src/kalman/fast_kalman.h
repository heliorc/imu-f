#pragma once
#include "includes.h"



typedef enum 
{
    PITCH = 0,
    YAW = 1,
    ROLL = 2
} filterAxisTypedef;

typedef struct fastKalman_s {
    float q;       //process noise covariance
    float r;       //measurement noise covariance
    float p;       //estimation error covariance matrix
    float k;       //kalman gain
    float x;       //state
    float lastX;   //previous state
    int gyroDfkfDataPtr;
    float gyroDfkfData[32];
} fastKalman_t;

typedef enum 
{
    NO_ESTIMATION = 0,
    STD_DEV_ESTIMATION = 1,
    DISTANCE_ESTIMATION = 2
} filterTypedef;

extern void fastKalmanInit(float q, float r, float p, float intialValue, filterTypedef type);
extern float fastKalmanUpdate(filterAxisTypeDef axis, float input);