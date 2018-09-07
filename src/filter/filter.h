#pragma once
#include "includes.h"
#include "biquad.h"

#define REFRESH_RATE           0.00003125f
#define DEFAULT_ROLL_Q         5000
#define DEFAULT_PITCH_Q        5000
#define DEFAULT_YAW_Q          3500
#define DEFAULT_ROLL_LPF_HZ    240
#define DEFAULT_PITCH_LPF_HZ   240
#define DEFAULT_YAW_LPF_HZ     240
#define DEFAULT_DYN_GAIN       20.0f
#define DEFAULT_FLAGS          0

typedef enum filterAxisTypedef {
    ROLL = 0,
    PITCH = 1,
    YAW = 2
} filterAxisTypedef_t;

typedef struct {
    uint32_t roll : 1;
    uint32_t pitch: 1;
    uint32_t yaw  : 1;
    uint32_t reserved: 29;
} fftFlags_t;

union fftFlags_u {
    fftFlags_t flags;
    uint32_t allFlags;
} fftFlags_u;

typedef struct filter_config
{
    uint16_t i_roll_q;
    uint16_t i_pitch_q;
    uint16_t i_yaw_q;
    uint16_t i_pitch_lpf_hz;
    uint16_t i_roll_lpf_hz;
    uint16_t i_yaw_lpf_hz;
    uint16_t w;
    union fftFlags_u fft;
    float roll_q;
    float pitch_q;
    float yaw_q;
    float pitch_lpf_hz;
    float roll_lpf_hz;
    float yaw_lpf_hz;
    float dyn_gain;
} filter_config_t;

extern volatile filter_config_t filterConfig;
//let other files be aware of these filters
extern biquad_axis_state_t axisX;
extern biquad_axis_state_t axisY;
extern biquad_axis_state_t axisZ;

extern void allow_filter_init(void);
extern void filter_init(void);
extern void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData);
