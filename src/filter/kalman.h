#pragma once
#include "includes.h"
#include "gyro.h"

#define WINDOW_SIZE 28
#define VARIANCE_SCALE 0.00033333f

typedef enum filterAxisTypedef {
    ROLL = 0,
    PITCH = 1,
    YAW = 2
} filterAxisTypedef_t;

typedef struct kalman
{
    float q;     //process noise covariance
    float r;     //measurement noise covariance
    float p;     //estimation error covariance matrix
    float k;     //kalman gain
    float x;     //state
    float lastX; //previous state
    uint32_t gyroKFDataPtr;
    float gyroKFData[WINDOW_SIZE];
} kalman_t;

typedef struct filter_config
{
    uint16_t i_roll_q;
    uint16_t i_pitch_q;
    uint16_t i_yaw_q;
    float roll_q;
    float pitch_q;
    float yaw_q;
} filter_config_t;

typedef struct variance
{
    float xVar;
    float yVar;
    float zVar;
    float xyCoVar;
    float xzCoVar;
    float yzCoVar;

    uint8_t windex;
    float xWindow[WINDOW_SIZE];
    float yWindow[WINDOW_SIZE];
    float zWindow[WINDOW_SIZE];

    float xSumMean;
    float ySumMean;
    float zSumMean;

    float xMean;
    float yMean;
    float zMean;

    float xSumVar;
    float ySumVar;
    float zSumVar;
    float xySumCoVar;
    float xzSumCoVar;
    float yzSumCoVar;

    float inverseN;

} variance_t;

extern volatile filter_config_t filterConfig;
extern variance_t varStruct;

extern kalman_t kalmanFilterStateRate[];

extern void kalman_init(void);
extern void kalman_update(volatile axisData_t *input, filteredData_t* output);