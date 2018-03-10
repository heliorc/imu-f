#include "includes.h"
#include "gyro.h"
#include "fast_kalman.h"


fastKalman_t fastKalmanFilterStateRate[3];
filter_type_t filterType;
volatile filter_config_t filterConfig;

void init_kalman(fastKalman_t *filter, float q, float r, float p, float intialValue)
{
    filter->q     = q * 0.001f; //add multiplier to make tuning easier
	filter->r     = r;    //add multiplier to make tuning easier
	filter->p     = p;    //add multiplier to make tuning easier
	filter->x     = intialValue;   //set intial value, can be zero if unknown
	filter->lastX = intialValue;   //set intial value, can be zero if unknown
	filter->k     = 0.0f;          //kalman gain,
    filter->gyroDfkfDataPtr = 0;  
}

void fast_kalman_init(filter_type_t type)
{
    filterType = type;
	init_kalman(&fastKalmanFilterStateRate[0], filterConfig.pitch_q, filterConfig.pitch_r, filterConfig.pitch_q, 0.0f);
	init_kalman(&fastKalmanFilterStateRate[1], filterConfig.roll_q,  filterConfig.roll_r,  filterConfig.roll_q,  0.0f);
	init_kalman(&fastKalmanFilterStateRate[2], filterConfig.yaw_q,   filterConfig.yaw_r,   filterConfig.yaw_q,   0.0f);
}

#pragma GCC push_options
#pragma GCC optimize ("O3")
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

static void _fast_kalman_pdate(fastKalman_t *axisFilter, float input)
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

float fast_kalman_pdate(filterAxisTypedef_t axis, float input)
{
    if (filterType == STD_DEV_ESTIMATION) {
        fastKalmanFilterStateRate[axis].r = noiseEstimate(fastKalmanFilterStateRate[axis].gyroDfkfData, 6);
    } else if (filterType == DISTANCE_ESTIMATION) {
        fastKalmanFilterStateRate[axis].r = distanceEstimate(fastKalmanFilterStateRate[axis].gyroDfkfData, 6);
    } 
    fastKalmanFilterStateRate[axis].gyroDfkfData[fastKalmanFilterStateRate[axis].gyroDfkfDataPtr] = input;
    _fast_kalman_pdate(&fastKalmanFilterStateRate[axis], input);
    fastKalmanFilterStateRate[axis].gyroDfkfDataPtr++;
	if (fastKalmanFilterStateRate[axis].gyroDfkfDataPtr > 5)
    {
		fastKalmanFilterStateRate[axis].gyroDfkfDataPtr=0;
    }
    return fastKalmanFilterStateRate[axis].x;
}
#pragma GCC pop_options