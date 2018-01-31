#pragma once
#include "includes.h"
#include "biquad.h"

extern biquad_state_t *lpfFilterStateRate;

extern void filter_data(axisData_t *gyroRateData, axisData_t *gyroAccData,float gyroTempData, filteredData_t *filteredData);
