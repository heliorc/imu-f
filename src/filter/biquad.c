#include "includes.h"
#include "biquad.h"


void biquad_init(float filterCutFreq, biquad_axis_state_t *newState, float refreshRateSeconds, uint32_t filterType, biquad_axis_state_t *oldState, float bandwidth)
{

	float samplingRate;
    float bigA, omega, sn, cs, alpha, beta;
    float a0, a1, a2, b0, b1, b2;

    float dbGain = 4.0;

    samplingRate = (1 / refreshRateSeconds);

	omega = 2 * (float)M_PI_FLOAT * (float) filterCutFreq / samplingRate;
	sn    = (float)sinf((float)omega);
	cs    = (float)cosf((float)omega);
	alpha = sn * (float)sinf( (float)((float)M_LN2_FLOAT / 2 * (float)bandwidth * (omega / sn)) );

	(void)(beta);
    //bigA  = powf(10, dbGain /40);
	//beta  = arm_sqrt_f32(bigA + bigA);

	switch (filterType)
	{
		case FILTER_TYPE_LOWPASS:
			b0 = (1 - cs) /2;
			b1 = 1 - cs;
			b2 = (1 - cs) /2;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
		case FILTER_TYPE_NOTCH:
			b0 = 1;
			b1 = -2 * cs;
			b2 = 1;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
		case FILTER_TYPE_PEEK:
		    bigA = powf(10, dbGain /40);
			b0   = 1 + (alpha * bigA);
			b1   = -2 * cs;
			b2   = 1 - (alpha * bigA);
			a0   = 1 + (alpha / bigA);
			a1   = -2 * cs;
			a2   = 1 - (alpha / bigA);
			break;
		 case FILTER_TYPE_HIGHPASS:
			b0 = (1 + cs) /2;
			b1 = -(1 + cs);
			b2 = (1 + cs) /2;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
	}

    // precompute the coefficients
    newState->a0 = b0 / a0;
    newState->a1 = b1 / a0;
    newState->a2 = b2 / a0;
    newState->a3 = a1 / a0;
    newState->a4 = a2 / a0;

    // zero initial samples
    //todo: make updateable on the fly
    newState->x1 =  oldState->x1;
    newState->x2 =  oldState->x2;
    newState->y1 =  oldState->y1;
    newState->y2 =  oldState->y1;

}

float biquad_update(float sample, biquad_axis_state_t *state)
{
    float result;

    /* compute result */
    result = state->a0 * (float)sample + state->a1 * state->x1 + state->a2 * state->x2 -
            state->a3 * state->y1 - state->a4 * state->y2;

    /* shift x1 to x2, sample to x1 */
    state->x2 = state->x1;
    state->x1 = (float)sample;
    /* shift y1 to y2, result to y1 */
    state->y2 = state->y1;
    state->y1 = result;

    return result;

}