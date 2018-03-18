#include "includes.h"
#include "gyro.h"
#include "fast_kalman.h"


fastKalman_t fastKalmanFilterStateRate[3];

volatile filter_config_t filterConfig;

void init_kalman(fastKalman_t *filter, float q, float r, float p, float intialValue)
{
	memset(filter, 0, sizeof(fastKalman_t));
    filter->q     = q * 0.001f; //add multiplier to make tuning easier
	filter->r     = r;    //add multiplier to make tuning easier
	filter->p     = p;    //add multiplier to make tuning easier
	filter->x     = intialValue;   //set intial value, can be zero if unknown
	filter->lastX = intialValue;   //set intial value, can be zero if unknown
}

void fast_kalman_init(void)
{
	init_kalman(&fastKalmanFilterStateRate[ROLL],  filterConfig.pitch_q, 88.0f, filterConfig.pitch_q, 0.0f);
	init_kalman(&fastKalmanFilterStateRate[PITCH], filterConfig.roll_q,  88.0f, filterConfig.roll_q,  0.0f);
	init_kalman(&fastKalmanFilterStateRate[YAW],   filterConfig.yaw_q,   88.0f, filterConfig.yaw_q,   0.0f);
}

#pragma GCC push_options
#pragma GCC optimize ("O3")
float variance(float data[], int size)
{
    float inverseN = 1.0f / (float)size;
    float sum = 0.0f, sum1 = 0.0f;
    float average, variance;
    int i;

    // sum of all elephants
    for (i = size-1; i >= 0; i--)
    {
        sum += data[i];
    }
    average = sum * inverseN;

    // find very ants
    for (i = size-1; i >= 0; i--)
    {
        sum1 += ((data[i] - average) * (data[i] - average));
    }
    variance = sum1 * inverseN;

    return variance;
    
}

float std_deviation(float data[], uint32_t size)
{
	float stdDev;
    arm_sqrt_f32( variance(data, size) , &stdDev);
    return stdDev;
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

static void _fast_kalman_update(fastKalman_t *axisFilter, float input)
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

float fast_kalman_update(filterAxisTypedef_t axis, float input, filter_type_t filterType)
{
	switch (filterType)
	{
		case STD_DEV_ESTIMATION:
			fastKalmanFilterStateRate[axis].r = std_deviation(fastKalmanFilterStateRate[axis].gyroDfkfData, filterConfig.filterWindow[axis]) * 0.001f;
			break;
		case VARIANCE_ESTIMATION:
			fastKalmanFilterStateRate[axis].r = variance(fastKalmanFilterStateRate[axis].gyroDfkfData, filterConfig.filterWindow[axis]) * 0.001f;
			break;
		case DISTANCE_ESTIMATION:
			fastKalmanFilterStateRate[axis].r = distanceEstimate(fastKalmanFilterStateRate[axis].gyroDfkfData, filterConfig.filterWindow[axis]);
			break;
		default:
		break;
	}

    if (filterType != NO_ESTIMATION)
	{
		fastKalmanFilterStateRate[axis].gyroDfkfData[fastKalmanFilterStateRate[axis].gyroDfkfDataPtr++] = input;
		if (fastKalmanFilterStateRate[axis].gyroDfkfDataPtr > filterConfig.filterWindow[axis]-1)
		{
			fastKalmanFilterStateRate[axis].gyroDfkfDataPtr=0;
		}
    }

    _fast_kalman_update(&fastKalmanFilterStateRate[axis], input);

    return fastKalmanFilterStateRate[axis].x;
}
#pragma GCC pop_options