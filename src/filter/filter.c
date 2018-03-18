#include "includes.h"
#include "biquad.h"
#include "filter.h"
#include "fast_kalman.h"
#include "imu.h" //for CONSTRAIN, doesn't really belong there, but oh well
//We should move constrain to utils.h and include it in includes

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
}

void filter_init_defaults(void)
{
	allowFilterInit                  = 1;
	filterConfig.i_pitch_q           = 500;
	filterConfig.i_pitch_w           = 6;
	filterConfig.i_roll_q            = 500;
	filterConfig.i_roll_w            = 6;
	filterConfig.i_yaw_q             = 350;
	filterConfig.i_yaw_w             = 6;
	filterConfig.i_pitch_lpf_hz      = 150;
	filterConfig.i_roll_lpf_hz       = 150;
	filterConfig.i_yaw_lpf_hz        = 150;
	filterConfig.pitch_q             = 500.0f;
	filterConfig.roll_q              = 500.0f;
	filterConfig.yaw_q               = 350.0f;
	filterConfig.pitch_lpf_hz        = 120.0f;
	filterConfig.roll_lpf_hz         = 120.0f;
	filterConfig.yaw_lpf_hz          = 120.0f;
	filterConfig.filterWindow[PITCH] = 6;
	filterConfig.filterWindow[ROLL]  = 6;
	filterConfig.filterWindow[YAW]   = 6;
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

		for (int x=0; x<3; x++)
		{
			//check window size for each axis
			if (filterConfig.filterWindow[x])
			{
				if(filterConfig.filterWindow[x] > 200)
				{
					filterConfig.filterWindow[x] = CONSTRAIN(filterConfig.filterWindow[x] - 200, 6, 30);
					filterConfig.filterType[x] = VARIANCE_ESTIMATION;
				}
				else if(filterConfig.filterWindow[x] > 100)
				{
					filterConfig.filterWindow[x] = CONSTRAIN(filterConfig.filterWindow[x] - 100, 6, 30);
					filterConfig.filterType[x] = DISTANCE_ESTIMATION;
				}
				else
				{
					filterConfig.filterWindow[x] = CONSTRAIN(filterConfig.filterWindow[x], 6, 30);
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

	filteredData->rateData.x = fast_kalman_update(ROLL, gyroRateData->x, filterConfig.filterType[ROLL]);
	filteredData->rateData.y = fast_kalman_update(PITCH, gyroRateData->y, filterConfig.filterType[PITCH]);
	filteredData->rateData.z = fast_kalman_update(YAW, gyroRateData->z, filterConfig.filterType[YAW]);

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