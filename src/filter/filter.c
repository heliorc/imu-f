#include "includes.h"
#include "biquad.h"
#include "fast_kalman.h"


void filter_data(axisData_t* gyroRateData, axisData_t* gyroAccData,float gyroTempData, filteredData_t* filteredData)
{
	static int firstRun = 1;

	if(firstRun)
	{
        firstRun = 0;
		fast_kalman_init(3000.0f, 88.0f, 3000.0f, 0.0f, DISTANCE_ESTIMATION);
        memset(&lpfFilterStateRate[0], 0, sizeof(biquad_state_t));
        memset(&lpfFilterStateRate[1], 0, sizeof(biquad_state_t));
        memset(&lpfFilterStateRate[2], 0, sizeof(biquad_state_t));
        #ifndef DEBUG
        biquad_init(150.0f, &lpfFilterStateRate[0], 0.00003125f, FILTER_TYPE_LOWPASS, &lpfFilterStateRate[0], BIQUAD_BANDWIDTH);
        biquad_init(150.0f, &lpfFilterStateRate[1], 0.00003125f, FILTER_TYPE_LOWPASS, &lpfFilterStateRate[0], BIQUAD_BANDWIDTH);
        biquad_init(150.0f, &lpfFilterStateRate[2], 0.00003125f, FILTER_TYPE_LOWPASS, &lpfFilterStateRate[0], BIQUAD_BANDWIDTH);
        #endif
	}

    #ifndef DEBUG
	filteredData->rateData.x = biquad_update( fast_kalman_pdate(0, gyroRateData->x), &lpfFilterStateRate[0]);
	filteredData->rateData.y = biquad_update( fast_kalman_pdate(1, gyroRateData->y), &lpfFilterStateRate[1]);
	filteredData->rateData.z = biquad_update( fast_kalman_pdate(2, gyroRateData->z), &lpfFilterStateRate[2]);
    #endif
    filteredData->rateData.x = fast_kalman_pdate(0, gyroRateData->x);
	filteredData->rateData.y = fast_kalman_pdate(1, gyroRateData->y);
	filteredData->rateData.z = fast_kalman_pdate(2, gyroRateData->z);


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