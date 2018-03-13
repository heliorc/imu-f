#include "includes.h"
#include "biquad.h"
#include "fft.h"
#include "imu.h" //for CONSTRAIN, doesn't really belong there, but oh well
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

void filter_init(void)
{
	fast_kalman_init();
	memset(&(lpfFilterStateRate.x), 0, sizeof(lpfFilterStateRate.x));
	memset(&(lpfFilterStateRate.y), 0, sizeof(lpfFilterStateRate.y));
	memset(&(lpfFilterStateRate.z), 0, sizeof(lpfFilterStateRate.z));

	biquad_init(filterConfig.pitch_lpf_hz, &(lpfFilterStateRate.x), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH, NULL);
	biquad_init(filterConfig.roll_lpf_hz, &(lpfFilterStateRate.y), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH, NULL);
	biquad_init(filterConfig.yaw_lpf_hz, &(lpfFilterStateRate.z), REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH, NULL);

	#ifndef DEBUG
	memset((biquad_axis_state_t *)&axisX, 0, sizeof(axisX));
	memset((biquad_axis_state_t *)&axisY, 0, sizeof(axisY));
	memset((biquad_axis_state_t *)&axisZ, 0, sizeof(axisZ));

	//init all of the notch biquads
	biquad_init(NOTCH_MAX, &axisX, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH, NULL);
	biquad_init(NOTCH_MAX, &axisY, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH, NULL);
	biquad_init(NOTCH_MAX, &axisZ, REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH, NULL);

	init_fft();
	#endif
}

void filter_init_defaults(void)
{
	//first run needs to init filters
	allowFilterInit               = 1;
	filterConfig.i_pitch_q        = 500;
	filterConfig.i_roll_q         = 500;
	filterConfig.i_yaw_q          = 350;
	filterConfig.i_pitch_lpf_hz   = 150;
	filterConfig.i_roll_lpf_hz    = 150;
	filterConfig.i_yaw_lpf_hz     = 150;
	filterConfig.i_dyn_gain       = 0;
	filterConfig.pitch_q          = 500.0f;
	filterConfig.roll_q           = 500.0f;
	filterConfig.yaw_q            = 350.0f;
	filterConfig.pitch_lpf_hz     = 150.0f;
	filterConfig.roll_lpf_hz      = 150.0f;
	filterConfig.yaw_lpf_hz       = 150.0f;
	filterConfig.dyn_gain         = 0.0f;
	filterConfig.filterWindow[0]  = 6;
	filterConfig.filterWindow[1]  = 6;
	filterConfig.filterWindow[2]  = 6;
}

void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData)
{
	static uint32_t dynFiltToUse = 0;

	if(allowFilterInit)
	{
		allowFilterInit = 0;
		//convert the ints to floats
		filterConfig.pitch_q        = (float)filterConfig.i_pitch_q;
		filterConfig.roll_q         = (float)filterConfig.i_roll_q;
		filterConfig.yaw_q          = (float)filterConfig.i_yaw_q;
		filterConfig.pitch_lpf_hz   = (float)filterConfig.i_pitch_lpf_hz;
		filterConfig.roll_lpf_hz    = (float)filterConfig.i_roll_lpf_hz;
		filterConfig.yaw_lpf_hz     = (float)filterConfig.i_yaw_lpf_hz;
		filterConfig.dyn_gain       = (float)(100.0f - (float)filterConfig.i_dyn_gain);

		for (int x=0; x<3; x++)
		{
			//check window size for each axis
			if (filterConfig.filterWindow[x])
			{
				if(filterConfig.filterWindow[x] > 100)
				{
					filterConfig.filterWindow[x] = CONSTRAIN(filterConfig.filterWindow[x] - 100, 6, 100);
					filterConfig.filterType[x] = DISTANCE_ESTIMATION;
				}
				else
				{
					filterConfig.filterWindow[x] = CONSTRAIN(filterConfig.filterWindow[x], 6, 100);
					filterConfig.filterType[x] = STD_DEV_ESTIMATION;
				}
			}
			else
			{
				filterConfig.filterType[x] = NO_ESTIMATION;
			}
		}
		filter_init();
	}

	filteredData->rateData.x = fast_kalman_update(0, gyroRateData->x, filterConfig.filterType[0]);
	filteredData->rateData.y = fast_kalman_update(1, gyroRateData->y, filterConfig.filterType[1]);
	filteredData->rateData.z = fast_kalman_update(2, gyroRateData->z, filterConfig.filterType[2]);

	#ifndef DEBUG
	if(filterConfig.i_dyn_gain)
	{
		//collect data for the FFT straight from the Kalman
		insert_gyro_data_for_fft(filteredData);
		filteredData->rateData.x = biquad_update(filteredData->rateData.x, &axisX);
		filteredData->rateData.y = biquad_update(filteredData->rateData.y, &axisY);
		filteredData->rateData.z = biquad_update(filteredData->rateData.z, &axisZ);
	}
	#endif

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