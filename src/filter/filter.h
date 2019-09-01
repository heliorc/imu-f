#pragma once
#include "includes.h"
#include "biquad.h"

#define REFRESH_RATE           0.00003125f
#define DEFAULT_ROLL_Q         3000
#define DEFAULT_PITCH_Q        3000
#define DEFAULT_YAW_Q          3000
#define BASE_LPF_HZ            70.0f


typedef enum filterAxisTypedef {
    ROLL = 0,
    PITCH = 1,
    YAW = 2
} filterAxisTypedef_t;

typedef struct filter_config
{
    uint16_t i_roll_q;
    uint16_t i_pitch_q;
    uint16_t i_yaw_q;
    uint16_t w;    
    float roll_q;
    float pitch_q;
    float yaw_q;
    float pitch_lpf_hz;
    float roll_lpf_hz;
    float yaw_lpf_hz;
    uint16_t acc_lpf_hz;
    uint16_t i_roll_lpf_hz;
	uint16_t i_pitch_lpf_hz;
	uint16_t i_yaw_lpf_hz;
	uint16_t r_weight;
} filter_config_t;

extern volatile filter_config_t filterConfig;
extern volatile uint32_t setPointNew;
extern volatile axisDataInt_t setPointInt;
extern volatile axisData_t setPoint;
extern void allow_filter_init(void);
extern void filter_init(void);
extern void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData);

void filter_acc(volatile axisData_t *gyroAccData);
