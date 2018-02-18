#include "includes.h"
#include "biquad.h"
#include "fft.h"
#include "filter.h"
#include "fast_kalman.h"

biquad_state_t lpfFilterStateRate;
biquad_state_t dynNotchStateRate[2];

volatile int allowFilterInit = 1;

void allow_filter_init(void)
{
	allowFilterInit = 1;
}

void filter_init(filter_type_t type)
{
	fast_kalman_init(type);
	memset(&(lpfFilterStateRate.x), 0, sizeof(biquad_axis_state_t));
	memset(&(lpfFilterStateRate.y), 0, sizeof(biquad_axis_state_t));
	memset(&(lpfFilterStateRate.z), 0, sizeof(biquad_axis_state_t));
	biquad_init(filterConfig.pitch_lpf_hz, &(lpfFilterStateRate.x), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
	biquad_init(filterConfig.roll_lpf_hz, &(lpfFilterStateRate.y), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
	biquad_init(filterConfig.yaw_lpf_hz, &(lpfFilterStateRate.z), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
	init_fft();
}

void filter_init_defaults(void)
{
	allowFilterInit = 0;
	filterConfig.i_pitch_q       = 3000;
	filterConfig.i_pitch_r       = 88;
	filterConfig.i_roll_q        = 3000;
	filterConfig.i_roll_r        = 88;
	filterConfig.i_yaw_q         = 1500;
	filterConfig.i_yaw_r         = 88;
	filterConfig.i_pitch_lpf_hz  = 120;
	filterConfig.i_roll_lpf_hz   = 120;
	filterConfig.i_yaw_lpf_hz    = 120;
	filterConfig.pitch_q         = 3000.0f;
	filterConfig.pitch_r         = 88.0f;
	filterConfig.roll_q          = 3000.0f;
	filterConfig.roll_r          = 88.0f;
	filterConfig.yaw_q           = 1500.0f;
	filterConfig.yaw_r           = 88.0f;
	filterConfig.pitch_lpf_hz    = 120.0f;
	filterConfig.roll_lpf_hz     = 120.0f;
	filterConfig.yaw_lpf_hz      = 120.0f;
	filter_init(NO_ESTIMATION);
}

void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData)
{
	static uint32_t dynFiltToUse = 0;

	if(allowFilterInit)
	{
		allowFilterInit = 0;
		//convert the ints to floats
		filterConfig.pitch_q        = (float)filterConfig.i_pitch_q;
		filterConfig.pitch_r        = (float)filterConfig.i_pitch_r;
		filterConfig.roll_q         = (float)filterConfig.i_roll_q;
		filterConfig.roll_r         = (float)filterConfig.i_roll_r;
		filterConfig.yaw_q          = (float)filterConfig.i_yaw_q;
		filterConfig.yaw_r          = (float)filterConfig.i_yaw_r;
		filterConfig.pitch_lpf_hz   = (float)filterConfig.i_pitch_lpf_hz;
		filterConfig.roll_lpf_hz    = (float)filterConfig.i_roll_lpf_hz;
		filterConfig.yaw_lpf_hz     = (float)filterConfig.i_yaw_lpf_hz;
		filter_init(NO_ESTIMATION);
	}

	filteredData->rateData.x = fast_kalman_pdate(0, gyroRateData->x);
	filteredData->rateData.y = fast_kalman_pdate(1, gyroRateData->y);
	filteredData->rateData.z = fast_kalman_pdate(2, gyroRateData->z);

	//collect data for the FFT straight from the Kalman
	fftGyroData[fftGyroDataInUse][0][fftGyroDataPtr]   = filteredData->rateData.x;
	fftGyroData[fftGyroDataInUse][1][fftGyroDataPtr]   = filteredData->rateData.y;
	fftGyroData[fftGyroDataInUse][2][fftGyroDataPtr++] = filteredData->rateData.z;
	if(fftGyroDataPtr==FFT_DATA_COLLECT_SIZE)
	{
		//don't allow overflow if main loops stops responding

		fftGyroDataPtr = 0;
	}

	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &(lpfFilterStateRate.x));
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &(lpfFilterStateRate.y));
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &(lpfFilterStateRate.z));

	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &(dynNotchStateRate[!fftGyroDataInUse].x));
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &(dynNotchStateRate[!fftGyroDataInUse].y));
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &(dynNotchStateRate[!fftGyroDataInUse].z));

	update_fft(&(filteredData->rateData), &fftStateRate);

	//no need to filter ACC is used in quaternions
	filteredData->accData.x  = gyroAccData->x;
	filteredData->accData.y  = gyroAccData->y;
	filteredData->accData.z  = gyroAccData->z;

	//should filter this
	filteredData->tempC       = gyroTempData;

}