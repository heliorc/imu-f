#pragma once
#include "includes.h"
#include "gyro.h"

typedef enum filterAxisTypedef
{
    PITCH = 0,
    ROLL= 1,
    YAW = 2,
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
    uint32_t i_pitch_q;
    uint32_t i_roll_q;
    uint32_t i_yaw_q;
    uint32_t i_pitch_lpf_hz;
    uint32_t i_roll_lpf_hz;
    uint32_t i_yaw_lpf_hz;
    uint32_t i_dyn_gain;
    float pitch_q;
    float roll_q;
    float yaw_q;
    float pitch_lpf_hz;
    float roll_lpf_hz;
    float yaw_lpf_hz;
    float dyn_gain;
    filter_type_t filterType[3];
    uint32_t filterWindow[3];
} filter_config_t;

extern volatile filter_config_t filterConfig;

extern fastKalman_t fastKalmanFilterStateRate[];

extern void fast_kalman_init(void);
extern float fast_kalman_update(filterAxisTypedef_t axis, float input, filter_type_t filterType);