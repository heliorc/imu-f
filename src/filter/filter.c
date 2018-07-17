#include "includes.h"
#include "gyro.h"
#include "filter.h"
#include "kalman.h"
#include "biquad.h"

volatile filter_config_t filterConfig = {
	DEFAULT_ROLL_Q,
	DEFAULT_PITCH_Q,
	DEFAULT_YAW_Q,
	DEFAULT_ROLL_LPF_HZ,
	DEFAULT_PITCH_LPF_HZ,
	DEFAULT_YAW_LPF_HZ,
	MIN_WINDOW_SIZE,
	(float)DEFAULT_ROLL_Q,
	(float)DEFAULT_PITCH_Q,
	(float)DEFAULT_YAW_Q,
	(float)DEFAULT_ROLL_LPF_HZ,
	(float)DEFAULT_PITCH_LPF_HZ,
	(float)DEFAULT_YAW_LPF_HZ,
};

biquad_state_t lpfFilterStateRate;

volatile int allowFilterInit = 1;

void allow_filter_init(void)
{
	allowFilterInit = 1;
}

void filter_init(void)
{
	kalman_init();

	biquad_init(filterConfig.roll_lpf_hz, &(lpfFilterStateRate.x), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
	biquad_init(filterConfig.pitch_lpf_hz, &(lpfFilterStateRate.y), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
	biquad_init(filterConfig.yaw_lpf_hz, &(lpfFilterStateRate.z), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
}

void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData)
{
	if(allowFilterInit)
	{
		allowFilterInit = 0;
		//convert the ints to floats
		filterConfig.roll_q         = (float)filterConfig.i_roll_q;
		filterConfig.pitch_q        = (float)filterConfig.i_pitch_q;
		filterConfig.yaw_q          = (float)filterConfig.i_yaw_q;
		filterConfig.roll_lpf_hz    = (float)filterConfig.i_roll_lpf_hz;
		filterConfig.pitch_lpf_hz   = (float)filterConfig.i_pitch_lpf_hz;
		filterConfig.yaw_lpf_hz     = (float)filterConfig.i_yaw_lpf_hz;
		filter_init();
	}

	kalman_update(gyroRateData, filteredData);

	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &(lpfFilterStateRate.x));
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &(lpfFilterStateRate.y));
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &(lpfFilterStateRate.z));

	//no need to filter ACC is used in quaternions
	filteredData->accData.x  = gyroAccData->x;
	filteredData->accData.y  = gyroAccData->y;
	filteredData->accData.z  = gyroAccData->z;

	//should filter this
	filteredData->tempC       = gyroTempData;

}