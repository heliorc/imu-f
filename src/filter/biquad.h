#pragma once
#include "includes.h"

#define M_LN2_FLOAT	0.69314718055994530942f
#define M_PI_FLOAT	3.14159265358979323846f
#define BIQUAD_BANDWIDTH 1.98f

#define FILTER_TYPE_LOWPASS  0
#define FILTER_TYPE_NOTCH    1
#define FILTER_TYPE_PEEK     2
#define FILTER_TYPE_HIGHPASS 3

typedef struct biquad_axis_state
{
    volatile float a0, a1, a2, a3, a4;
    volatile float x1, x2, y1, y2;
} biquad_axis_state_t;

typedef struct biquad_state
{
    biquad_axis_state_t x;
    biquad_axis_state_t y;
    biquad_axis_state_t z;
} biquad_state_t;

extern void  biquad_init(float filterCutFreq, biquad_axis_state_t *newState, float refreshRateSeconds, uint32_t filterType, float bandwidth, biquad_axis_state_t *oldState);
extern float biquad_update(float sample, biquad_axis_state_t *state);
