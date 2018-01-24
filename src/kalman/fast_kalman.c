#include <math.h>
#include "fast_kalman.h"
#include "gyro.h"
#include "includes.h"


fastKalman_t filter[3];
filterTypedef_t filterType;

void initFilter(fastKalman_t *filter, float q, float r, float p, float intialValue) {
    filter->q     = q * 0.001f; //add multiplier to make tuning easier
	filter->r     = r;    //add multiplier to make tuning easier
	filter->p     = p;    //add multiplier to make tuning easier
	filter->x     = intialValue;   //set intial value, can be zero if unknown
	filter->lastX = intialValue;   //set intial value, can be zero if unknown
	filter->k     = 0.0f;          //kalman gain,
    filter->gyroDfkfDataPtr = 0;  
}

void fastKalmanInit(float q, float r, float p, float intialValue, filterTypedef_t type)
{
    filterType = type;
	initFilter(&filter[0], q, r, p, intialValue);
	initFilter(&filter[1], q, r, p, intialValue);
	initFilter(&filter[2], q, r, p, intialValue);
}


float noiseEstimate(float data[], uint32_t size)
{
    uint32_t i;
    float sum = 0.0, sumOfSquares = 0.0, stdDev;
    for (int i = size+1; i >= 0; i--)
    {
        sum += data[i];
        sumOfSquares += powf(data[i], 2);
    }

    arm_sqrt_f32((sumOfSquares-powf(sum, 2))/(float)size, &stdDev);
    return (stdDev * 0.003f);
}

float distanceEstimate(float data[], uint32_t size)
{
	float largest = -10000.0f;
	float smallest = 10000.0f;
	float range;

	//find the range
	for(int x=size;x>=0;x--)
	{
		if(data[x]>largest)
			largest = data[x];

		if(data[x]<smallest)
			smallest = data[x];
	}

	range = largest - smallest;

	return (range * 0.001f);
}

static void _fastKalmanUpdate(fastKalman_t *axisFilter, float input)
{
    //project the state ahead using acceleration
    axisFilter->x += (axisFilter->x - axisFilter->lastX);

    //update last state
    axisFilter->lastX = axisFilter->x;

	//prediction update
	axisFilter->p = axisFilter->p + axisFilter->q;

	//measurement update
	axisFilter->k = axisFilter->p / (axisFilter->p + axisFilter->r);
	axisFilter->x += axisFilter->k * (input - axisFilter->x);
	axisFilter->p = (1.0f - axisFilter->k) * axisFilter->p;
}

//for(int x=2;x>=0;x--)
//{
//	fastKalmanUpdate(&filter[x], input[x]);
//}

float fastKalmanUpdate(filterAxisTypedef_t axis, float input)
{
    if (filterType == STD_DEV_ESTIMATION) {
        filter[axis].r = noiseEstimate(filter[axis].gyroDfkfData, 6);
    } else if (filterType == DISTANCE_ESTIMATION) {
        filter[axis].r = distanceEstimate(filter[axis].gyroDfkfData, 6);
    } 
    filter[axis].gyroDfkfData[filter[axis].gyroDfkfDataPtr] = input;
    _fastKalmanUpdate(&filter[axis], input);
    filter[axis].gyroDfkfDataPtr++;
	if (filter[axis].gyroDfkfDataPtr > 5)
    {
		filter[axis].gyroDfkfDataPtr=0;
    }
    return filter[axis].x;
}

void filter_data(axisData_t gyroRateData, axisData_t gyroAccData,float gyroTempData, filteredData_t* filteredData)
{
	//what's insie the filteredData_t typedef
	//float rateData[3];
    //float accData[3];
    //float tempC;
    //float quaternion[4];

	//filter data here

	//set filterData
	filteredData->rateData.x = gyroRateData.x;
	filteredData->rateData.y = gyroRateData.y;
	filteredData->rateData.z = gyroRateData.z;

	filteredData->accData.x  = gyroAccData.x;
	filteredData->accData.y  = gyroAccData.y;
	filteredData->accData.z  = gyroAccData.z;

	filteredData->tempC       = gyroTempData;

}