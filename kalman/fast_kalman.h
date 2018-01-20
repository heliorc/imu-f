#pragma once
#include <math.h>

typedef struct fastKalman_s {
    float q;      
    float r;       
    float p;       
    float k;       
    float x;       
    float lastX;   
} fastKalman_t;

extern float noiseEstimate(float data[], uint32_t size);
extern void fastKalmanInit(fastKalman_t *filter, float q, float r, float p, float intialValue);
extern float fastKalmanUpdate(fastKalman_t *filter, float input);
