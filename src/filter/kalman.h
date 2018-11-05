#pragma once
#include "includes.h"
#include "gyro.h"
#include "filter.h"

#define MAX_WINDOW_SIZE 300
#define DEF_WINDOW_SIZE 32
#define MIN_WINDOW_SIZE 6

// #define VARIANCE_SCALE 0.001
#define VARIANCE_SCALE 0.3333333f

typedef struct kalman
{
    float q;     //process noise covariance
    float r;     //measurement noise covariance
    float p;     //estimation error covariance matrix
    float k;     //kalman gain
    float x;     //state
    float lastX; //previous state
    float e;
} kalman_t;

typedef struct variance
{
    float xVar;
    float yVar;
    float zVar;
    float xyCoVar;
    float xzCoVar;
    float yzCoVar;

    uint32_t windex;
    float xWindow[MAX_WINDOW_SIZE];
    float yWindow[MAX_WINDOW_SIZE];
    float zWindow[MAX_WINDOW_SIZE];

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

extern void kalman_init(void);
extern void kalman_update(volatile axisData_t *input, filteredData_t* output);
