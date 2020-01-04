#include "includes.h"
#include "gyro.h"
#include "filter.h"
#include "kalman.h"
#include "biquad.h"

/*
typedef struct filter_config
{
    uint16_t i_roll_q;
    uint16_t i_pitch_q;
    uint16_t i_yaw_q;
    uint16_t w;

    float roll_q;
    float pitch_q;
    float yaw_q;

    float pitch_lpf_hz;
    float roll_lpf_hz;
    float yaw_lpf_hz;

    uint16_t acc_lpf_hz;


    uint16_t i_roll_lpf_hz;
	uint16_t i_pitch_lpf_hz;
	uint16_t i_yaw_lpf_hz;
} filter_config_t;
*/
volatile filter_config_t filterConfig =
{
		DEFAULT_ROLL_Q,
		DEFAULT_PITCH_Q,
		DEFAULT_YAW_Q,
		MIN_WINDOW_SIZE,

		(float)DEFAULT_ROLL_Q,
		(float)DEFAULT_PITCH_Q,
		(float)DEFAULT_YAW_Q,

		(float)BASE_LPF_HZ,
		(float)BASE_LPF_HZ,
		(float)BASE_LPF_HZ,

		40.0f,

		BASE_LPF_HZ,
		BASE_LPF_HZ,
		BASE_LPF_HZ,
		100,
};

// PT1 Low Pass filter
bool acc_filter_initialized = false;
typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

pt1Filter_t ax_filter;
pt1Filter_t ay_filter;
pt1Filter_t az_filter;

float pt1FilterGain(uint16_t f_cut, float dT);
void  pt1FilterInit(pt1Filter_t *filter, float k, float val);
float pt1FilterApply(pt1Filter_t *filter, float input);

biquad_state_t lpfFilterStateRate;
volatile uint32_t setPointNew;
volatile axisDataInt_t setPointInt;
volatile axisData_t oldSetPoint;
volatile axisData_t setPoint;
volatile int allowFilterInit = 1;

extern float r_filter_weight;

void allow_filter_init(void)
{
	allowFilterInit = 1;
}

void filter_biquad_init(float freq, biquad_axis_state_t *filterState)
{
	biquad_init(freq, filterState, REFRESH_RATE, FILTER_TYPE_LOWPASS, BIQUAD_BANDWIDTH);
}

void filter_init(void)
{
//#define ACC_CUTOFF    (60.0f)
#define ACC_READ_RATE (1.0f / 1000.0f)

	memset((uint32_t *)&setPoint, 0, sizeof(axisData_t));
	memset((uint32_t *)&oldSetPoint, 0, sizeof(axisData_t));
	memset((uint32_t *)&setPointInt, 0, sizeof(axisDataInt_t));
	kalman_init();
	filter_biquad_init(filterConfig.i_roll_lpf_hz, &(lpfFilterStateRate.x));
	filter_biquad_init(filterConfig.i_pitch_lpf_hz, &(lpfFilterStateRate.y));
	filter_biquad_init(filterConfig.i_yaw_lpf_hz, &(lpfFilterStateRate.z));

	// set imuf acc cutoff frequency
	const float k = pt1FilterGain((float)filterConfig.acc_lpf_hz, ACC_READ_RATE);
	pt1FilterInit(&ax_filter, k, 0.0f);
	pt1FilterInit(&ay_filter, k, 0.0f);
	pt1FilterInit(&az_filter, k, 0.0f);

	r_filter_weight = (float)filterConfig.r_weight / 100.0f;
}

void filter_data(volatile axisData_t *gyroRateData, volatile axisData_t *gyroAccData, float gyroTempData, filteredData_t *filteredData)
{
	if (allowFilterInit)
	{
		allowFilterInit = 0;
		//convert the ints to floats
		filterConfig.roll_q = (float)filterConfig.i_roll_q;
		filterConfig.pitch_q = (float)filterConfig.i_pitch_q;
		filterConfig.yaw_q = (float)filterConfig.i_yaw_q;
		filter_init();
	}

	if (setPointNew)
	{
		memcpy((uint32_t *)&setPoint, (uint32_t *)&setPointInt, sizeof(axisData_t));
	}

	kalman_update(gyroRateData, filteredData);

	filteredData->rateData.x = biquad_update(filteredData->rateData.x, &(lpfFilterStateRate.x));
	filteredData->rateData.y = biquad_update(filteredData->rateData.y, &(lpfFilterStateRate.y));
	filteredData->rateData.z = biquad_update(filteredData->rateData.z, &(lpfFilterStateRate.z));

	if (setPointNew)
	{
		setPointNew = 0;
		if (oldSetPoint.x != setPoint.x)
		{
			filterConfig.roll_lpf_hz = CONSTRAIN(filterConfig.i_roll_lpf_hz * ABS(1.0f - (setPoint.x / filteredData->rateData.x)), 10.0f, 500.0f);
			filter_biquad_init(filterConfig.roll_lpf_hz, &(lpfFilterStateRate.x));
		}
		if (oldSetPoint.y != setPoint.y)
		{
			filterConfig.pitch_lpf_hz = CONSTRAIN(filterConfig.i_pitch_lpf_hz * ABS(1.0f - (setPoint.x / filteredData->rateData.x)), 10.0f, 500.0f);
			filter_biquad_init(filterConfig.pitch_lpf_hz, &(lpfFilterStateRate.y));
		}
		if (oldSetPoint.z != setPoint.z)
		{
			filterConfig.yaw_lpf_hz = CONSTRAIN(filterConfig.i_yaw_lpf_hz * ABS(1.0f - (setPoint.x / filteredData->rateData.x)), 10.0f, 500.0f);
			filter_biquad_init(filterConfig.yaw_lpf_hz, &(lpfFilterStateRate.z));
		}
		memcpy((uint32_t *)&oldSetPoint, (uint32_t *)&setPoint, sizeof(axisData_t));
	}
	//no need to filter ACC is used in quaternions
	filteredData->accData.x = gyroAccData->x;
	filteredData->accData.y = gyroAccData->y;
	filteredData->accData.z = gyroAccData->z;

	//should filter this
	filteredData->tempC = gyroTempData;
}

float pt1FilterGain(uint16_t f_cut, float dT)
{
    float RC = 1 / ( 2 * M_PI_FLOAT * f_cut);
    return dT / (RC + dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k, float val)
{
    filter->state = val;
    filter->k = k;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

void filter_acc(volatile axisData_t *gyroAccData)
{
	if (!acc_filter_initialized)
	{
		acc_filter_initialized = true;
		ax_filter.state = gyroAccData->x;
		ay_filter.state = gyroAccData->y;
		az_filter.state = gyroAccData->z;
	}
	else
	{
		 gyroAccData->x = pt1FilterApply(&ax_filter,  gyroAccData->x);
		 gyroAccData->y = pt1FilterApply(&ay_filter,  gyroAccData->y);
		 gyroAccData->z = pt1FilterApply(&az_filter,  gyroAccData->z);
	}
}
