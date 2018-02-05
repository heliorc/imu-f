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

typedef struct filter_config {
    float pitch_q;
    float pitch_r;
    float roll_q;
    float roll_r;
    float yaw_q;
    float yaw_r;
} filter_config_t;

extern filter_config_t filterConfig;

extern fastKalman_t fastKalmanFilterStateRate[];

extern void fast_kalman_init(filterTypedef_t type);
extern float fast_kalman_pdate(filterAxisTypedef_t axis, float input);