#pragma once

#include "filter.h"
#include "arm_math.h"

#define FFT_SIZE              32
#define FFT_MIN_HZ            100 
#define FFT_MAX_HZ            500 
#define NOTCH_WIDTH           40 
#define NOTCH_MIN             100 
#define NOTCH_MAX             460 
#define AXIS_AMOUNT           3
#define FFT_BUFFS             2
#define FFT_DATA_COLLECT_SIZE 42 //allow overflow room since FFT calcs aren't realtime
#define NOTCH_APROX(cen, max) ((float)(max * cen) / ((float)(cen - max) * (float)(cen + max)))

#define FFT_DATA_SET_SIZE 96
//#define BQQ 0.7071067811865475f //butterworth 1/sqrt(2)
#define BQQ 1.0f //butterworth 1/sqrt(2)

typedef enum fftUpdateState
{
    FFT_STATE_CALCULATE_X = 0,
    FFT_STATE_CALCULATE_X_DONE = 1,
    FFT_STATE_CALCULATE_Y = 2,
    FFT_STATE_CALCULATE_Y_DONE = 3,
    FFT_STATE_CALCULATE_Z = 4,
    FFT_STATE_CALCULATE_Z_DONE = 5,
} fftUpdateState_t;

typedef struct fft_data {
    float max;
    float cen;
    float cutoffFreq;
    float notchQ;
} fft_data_t;

extern void init_fft(void);
extern void update_fft(void);
extern void increment_fft_state(void);
extern void insert_gyro_data_for_fft(filteredData_t* filteredData);