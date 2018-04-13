#include "includes.h"
#include "filter.h"
#include "kalman.h"
#include "imu.h" //for CONSTRAIN, doesn't really belong there, but oh well
//We should move constrain to utils.h and include it in includes

volatile int allowFilterInit = 1;

void allow_filter_init(void)
{
	allowFilterInit = 1;
}

void filter_init(void)
{
	kalman_init();
}

void filter_init_defaults(void)
{
	allowFilterInit                  = 1;
	filterConfig.i_pitch_q           = 500;
	filterConfig.i_roll_q            = 500;
	filterConfig.i_yaw_q             = 350;
	filterConfig.pitch_q             = 500.0f;
	filterConfig.roll_q              = 500.0f;
	filterConfig.yaw_q               = 350.0f;
}

void filter_data(volatile axisData_t* gyroRateData, volatile axisData_t* gyroAccData, float gyroTempData, filteredData_t* filteredData)
{
	if(allowFilterInit)
	{
		allowFilterInit = 0;
		//convert the ints to floats
		filterConfig.pitch_q        = (float)filterConfig.i_pitch_q;
		filterConfig.roll_q         = (float)filterConfig.i_roll_q;
		filterConfig.yaw_q          = (float)filterConfig.i_yaw_q;
		filter_init();
	}

	kalman_update(gyroRateData, filteredData);

	//no need to filter ACC is used in quaternions
	filteredData->accData.x  = gyroAccData->x;
	filteredData->accData.y  = gyroAccData->y;
	filteredData->accData.z  = gyroAccData->z;

	//should filter this
	filteredData->tempC       = gyroTempData;

}