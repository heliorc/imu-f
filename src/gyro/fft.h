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

extern void init_fft(void);
extern void update_fft(void);
extern void increment_fft_state(void);
extern void insert_gyro_data_for_fft(filteredData_t* filteredData);