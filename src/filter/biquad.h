#pragma once
#include "includes.h"

#define M_LN2_FLOAT	0.69314718055994530942f
#define M_PI_FLOAT	3.14159265358979323846f
#define BIQUAD_BANDWIDTH 1.98f

#define FILTER_TYPE_LOWPASS  0
#define FILTER_TYPE_NOTCH    1
#define FILTER_TYPE_PEEK     2
#define FILTER_TYPE_HIGHPASS 3

typedef struct biquad_state
{
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
} biquad_state_t;

extern biquad_state_t lpfFilterStateRate[];

extern void  biquad_init(float filterCutFreq, biquad_state_t *newState, float refreshRateSeconds, uint32_t filterType, biquad_state_t *oldState, float bandwidth);
extern float biquad_update(float sample, biquad_state_t *bQstate);
