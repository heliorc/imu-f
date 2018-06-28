#pragma once
#include "includes.h"
#include "gyro.h"
#include "filter.h"

#define MAX_WINDOW_SIZE 300
#define DEF_WINDOW_SIZE 32
#define MIN_WINDOW_SIZE 6

// #define VARIANCE_SCALE 1.0f
#define VARIANCE_SCALE 0.5f

typedef struct kalman
{
    float q;     //process noise covariance
    float r;     //measurement noise covariance
    float p;     //estimation error covariance matrix
    float k;     //kalman gain
    float x;     //state
    float acc;   //acceleration
    float lastX; //previous state
    float e;
} kalman_t;

typedef struct variance
{
    float xVar;
    float yVar;
    float zVar;

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

    float inverseN;
} variance_t;

extern volatile filter_config_t filterConfig;
extern volatile uint32_t setPointNew;
extern volatile axisDataInt_t setPointInt;
extern volatile axisData_t setPoint;
extern void kalman_init(void);
extern void kalman_update(volatile axisData_t *input, filteredData_t* output);
