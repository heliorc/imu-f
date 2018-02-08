#pragma once
#include "includes.h"
#include "biquad.h"

extern biquad_state_t lpfFilterStateRate;

extern void filter_init(void);
extern void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData);
