#include "includes.h"
#include "gyro.h"
#include "filter.h"
#include "kalman.h"
#include "biquad.h"
#include "fft.h"

volatile filter_config_t filterConfig = {
	DEFAULT_ROLL_Q,
	DEFAULT_PITCH_Q,
	DEFAULT_YAW_Q,
	DEFAULT_ROLL_LPF_HZ,
	DEFAULT_PITCH_LPF_HZ,
	DEFAULT_YAW_LPF_HZ,
	MIN_WINDOW_SIZE,
	.fft = {
		.allFlags = DEFAULT_FLAGS,
	},
	(float)DEFAULT_ROLL_Q,
	(float)DEFAULT_PITCH_Q,
	(float)DEFAULT_YAW_Q,
	(float)DEFAULT_ROLL_LPF_HZ,
	(float)DEFAULT_PITCH_LPF_HZ,
	(float)DEFAULT_YAW_LPF_HZ,
	(float)DEFAULT_DYN_GAIN,
};

biquad_state_t lpfFilterStateRate;

//actual dynamic filters live here
biquad_axis_state_t axisX;
biquad_axis_state_t axisY;
biquad_axis_state_t axisZ;

volatile int allowFilterInit = 1;

void allow_filter_init(void)
{
	allowFilterInit = 1;
}

void filter_init(void)
{
	kalman_init();
	memset(&(lpfFilterStateRate.x), 0, sizeof(lpfFilterStateRate.x));
	memset(&(lpfFilterStateRate.y), 0, sizeof(lpfFilterStateRate.y));
	memset(&(lpfFilterStateRate.z), 0, sizeof(lpfFilterStateRate.z));

	biquad_init(filterConfig.roll_lpf_hz, &(lpfFilterStateRate.x), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
	biquad_init(filterConfig.pitch_lpf_hz, &(lpfFilterStateRate.y), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
	biquad_init(filterConfig.yaw_lpf_hz, &(lpfFilterStateRate.z), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);

	#ifndef DEBUG
	memset((biquad_axis_state_t *)&axisX, 0, sizeof(axisX));
	memset((biquad_axis_state_t *)&axisY, 0, sizeof(axisY));
	memset((biquad_axis_state_t *)&axisZ, 0, sizeof(axisZ));

	//init all of the notch biquads
	if (filterConfig.fft.flags.roll) {
		biquad_init(NOTCH_MAX, &axisX, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
	}
	if (filterConfig.fft.flags.pitch) {
		biquad_init(NOTCH_MAX, &axisY, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
	}
	if (filterConfig.fft.flags.yaw) {
		biquad_init(NOTCH_MAX, &axisZ, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
	}

	init_fft();
	#endif
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
		filterConfig.dyn_gain       = (float)filterConfig.dyn_gain;
		filter_init();
	}

	kalman_update(gyroRateData, filteredData);

	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &(lpfFilterStateRate.x));
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &(lpfFilterStateRate.y));
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &(lpfFilterStateRate.z));
	#ifndef DEBUG
	//collect data for the FFT straight from the Kalman
	insert_gyro_data_for_fft(filteredData);
	if ((filterConfig.fft.allFlags & 0x07)) {
		if (filterConfig.fft.flags.roll) {
			filteredData->rateData.x = biquad_update(filteredData->rateData.x, &axisX);
		}
		if (filterConfig.fft.flags.roll) {
			filteredData->rateData.y = biquad_update(filteredData->rateData.y, &axisY);
		}
		if (filterConfig.fft.flags.roll) {
			filteredData->rateData.z = biquad_update(filteredData->rateData.z, &axisZ);
		}
	}
	
	#endif	

	//no need to filter ACC is used in quaternions
	filteredData->accData.x  = gyroAccData->x;
	filteredData->accData.y  = gyroAccData->y;
	filteredData->accData.z  = gyroAccData->z;

	//should filter this
	filteredData->tempC       = gyroTempData;

}