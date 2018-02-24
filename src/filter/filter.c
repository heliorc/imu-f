#include "includes.h"
#include "biquad.h"
#include "fft.h"
#include "filter.h"
#include "fast_kalman.h"

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

void filter_init(filter_type_t type)
{
	fast_kalman_init(type);
	memset(&(lpfFilterStateRate.x), 0, sizeof(lpfFilterStateRate.x));
	memset(&(lpfFilterStateRate.y), 0, sizeof(lpfFilterStateRate.y));
	memset(&(lpfFilterStateRate.z), 0, sizeof(lpfFilterStateRate.z));

	biquad_init(filterConfig.pitch_lpf_hz, &(lpfFilterStateRate.x), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH, NULL);
	biquad_init(filterConfig.roll_lpf_hz, &(lpfFilterStateRate.y), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH, NULL);
	biquad_init(filterConfig.yaw_lpf_hz, &(lpfFilterStateRate.z), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH, NULL);

	memset((biquad_axis_state_t *)&axisX, 0, sizeof(axisX));
	memset((biquad_axis_state_t *)&axisY, 0, sizeof(axisY));
	memset((biquad_axis_state_t *)&axisZ, 0, sizeof(axisZ));

    //init all of the notch biquads
    biquad_init(NOTCH_MAX, &axisX, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH, NULL);
    biquad_init(NOTCH_MAX, &axisY, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH, NULL);
    biquad_init(NOTCH_MAX, &axisZ, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH, NULL);

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
	insert_gyro_data_for_fft(filteredData);

	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &(lpfFilterStateRate.x));
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &(lpfFilterStateRate.y));
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &(lpfFilterStateRate.z));

	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &axisX);
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &axisY);
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &axisZ);

	//no need to filter ACC is used in quaternions
	filteredData->accData.x  = gyroAccData->x;
	filteredData->accData.y  = gyroAccData->y;
	filteredData->accData.z  = gyroAccData->z;

	//should filter this
	filteredData->tempC       = gyroTempData;

}