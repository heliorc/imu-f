#pragma once
#include "includes.h"
#include "biquad.h"
#include "fast_kalman.h"

#define REFRESH_RATE 0.00003125f

extern biquad_state_t dynNotchStateRate;

//let other files be aware of these filters
extern biquad_axis_state_t axisX;
extern biquad_axis_state_t axisY;
extern biquad_axis_state_t axisZ;

extern void allow_filter_init(void);
extern void filter_init(filter_type_t type);
extern void filter_init_defaults(void);
extern void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData);
