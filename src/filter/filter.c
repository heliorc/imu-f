#include "includes.h"
#include "biquad.h"
#include "fast_kalman.h"

biquad_state_t lpfFilterStateRate;

void filter_init(filter_type_t type)
{
	fast_kalman_init(type);
	memset(&(lpfFilterStateRate.x), 0, sizeof(biquad_axis_state_t));
	memset(&(lpfFilterStateRate.y), 0, sizeof(biquad_axis_state_t));
	memset(&(lpfFilterStateRate.z), 0, sizeof(biquad_axis_state_t));
	#ifndef DEBUG
	biquad_init(150.0f, &(lpfFilterStateRate.x), 0.00003125f, FILTER_TYPE_LOWPASS, &(lpfFilterStateRate.x), BIQUAD_BANDWIDTH);
	biquad_init(150.0f, &(lpfFilterStateRate.y), 0.00003125f, FILTER_TYPE_LOWPASS, &(lpfFilterStateRate.y), BIQUAD_BANDWIDTH);
	biquad_init(150.0f, &(lpfFilterStateRate.z), 0.00003125f, FILTER_TYPE_LOWPASS, &(lpfFilterStateRate.z), BIQUAD_BANDWIDTH);
	#endif
}

void filter_init_defaults(void)
{
	filterConfig.pitch_q  = 3000.0f;
	filterConfig.pitch_r  = 88.0f;
	filterConfig.roll_q   = 3000.0f;
	filterConfig.roll_r   = 88.0f;
	filterConfig.yaw_q    = 1500.0f;
	filterConfig.yaw_r    = 88.0f;
	filter_init(NO_ESTIMATION);
}

void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData)
{

	filteredData->rateData.x = fast_kalman_pdate(0, gyroRateData->x);
	filteredData->rateData.y = fast_kalman_pdate(1, gyroRateData->y);
	filteredData->rateData.z = fast_kalman_pdate(2, gyroRateData->z);
    #ifndef DEBUG
	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &(lpfFilterStateRate.x));
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &(lpfFilterStateRate.y));
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &(lpfFilterStateRate.z));
    #endif

	//what's insie the filteredData_t typedef
	//float rateData[3];
    //float accData[3];
    //float tempC;
    //float quaternion[4];

	//filter data here

	//no need to filter ACC is used in quaternions
	filteredData->accData.x  = gyroAccData->x;
	filteredData->accData.y  = gyroAccData->y;
	filteredData->accData.z  = gyroAccData->z;

	//should filter this
	filteredData->tempC       = gyroTempData;

}