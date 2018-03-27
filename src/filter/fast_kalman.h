#pragma once
#include "includes.h"
#include "gyro.h"

typedef enum filterAxisTypedef
{
    ROLL = 0,
    PITCH = 1,
    YAW = 2
} filterAxisTypedef_t;

typedef struct fastKalman {
    float q;       //process noise covariance
    float r;       //measurement noise covariance
    float p;       //estimation error covariance matrix
    float k;       //kalman gain
    float x;       //state
    float lastX;   //previous state
    uint32_t gyroDfkfDataPtr;
    float gyroDfkfData[100];
} fastKalman_t;

typedef enum filter_type
{
    NO_ESTIMATION = 0,
    STD_DEV_ESTIMATION = 1,
    DISTANCE_ESTIMATION = 2,
    VARIANCE_ESTIMATION = 3
} filter_type_t;

typedef struct filter_config {
    uint16_t i_pitch_q;
    uint16_t i_pitch_w;
    uint16_t i_roll_q;
    uint16_t i_roll_w;
    uint16_t i_yaw_q;
    uint16_t i_yaw_w;
    uint16_t i_pitch_lpf_hz;
    uint16_t i_roll_lpf_hz;
    uint16_t i_yaw_lpf_hz;
    float pitch_q;
    float roll_q;
    float yaw_q;
    float pitch_lpf_hz;
    float roll_lpf_hz;
    float yaw_lpf_hz;
    filter_type_t filterType[3];
    uint32_t filterWindow[3];
} filter_config_t;

extern volatile filter_config_t filterConfig;

extern fastKalman_t fastKalmanFilterStateRate[];

extern void fast_kalman_init(void);
extern float fast_kalman_update(filterAxisTypedef_t axis, float input, filter_type_t filterType);