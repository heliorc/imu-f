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

typedef enum filter_type
{
    NO_ESTIMATION = 0,
    STD_DEV_ESTIMATION = 1,
    DISTANCE_ESTIMATION = 2
} filter_type_t;

typedef struct filter_config {
    uint16_t i_pitch_q;
    uint16_t i_pitch_r;
    uint16_t i_roll_q;
    uint16_t i_roll_r;
    uint16_t i_yaw_q;
    uint16_t i_yaw_r;
    uint16_t i_pitch_lpf_hz;
    uint16_t i_roll_lpf_hz;
    uint16_t i_yaw_lpf_hz;
    float pitch_q;
    float pitch_r;
    float roll_q;
    float roll_r;
    float yaw_q;
    float yaw_r;
    float pitch_lpf_hz;
    float roll_lpf_hz;
    float yaw_lpf_hz;
} filter_config_t;

extern filter_config_t filterConfig;

extern fastKalman_t fastKalmanFilterStateRate[];

extern void fast_kalman_init(filter_type_t type);
extern float fast_kalman_pdate(filterAxisTypedef_t axis, float input);