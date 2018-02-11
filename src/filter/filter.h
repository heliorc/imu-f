#pragma once
#include "includes.h"
#include "biquad.h"
#include "fast_kalman.h"

extern biquad_state_t lpfFilterStateRate;

extern void allow_filter_init(void);
extern void filter_init(filter_type_t type);
extern void filter_init_defaults(void);
extern void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData);
