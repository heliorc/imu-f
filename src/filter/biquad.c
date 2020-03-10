#include "includes.h"
#include "biquad.h"


void biquad_init(float filterCutFreq, biquad_axis_state_t *state, float refreshRateSeconds, uint32_t filterType, float bandwidth)
{

	float samplingRate;
    float bigA, omega, sn, cs, alpha, beta;
    float a0, a1, a2, b0, b1, b2;

    float dbGain = 4.0;

    samplingRate = (1.0f / refreshRateSeconds);

	omega = 2.0f * (float)M_PI_FLOAT * (float) filterCutFreq / samplingRate;
	sn    = (float)sinf((float)omega);
	cs    = (float)cosf((float)omega);
	alpha = sn * (float)sinf( (float)((float)M_LN2_FLOAT / 2.0f * (float)bandwidth * (omega / sn)) );

	(void)(beta);

	switch (filterType)
	{
		case FILTER_TYPE_LOWPASS:
			b0 = (1.0f - cs) * 0.5f;
			b1 = 1.0f - cs;
			b2 = (1.0f - cs) * 0.5f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * cs;
			a2 = 1.0f - alpha;
			break;
		case FILTER_TYPE_NOTCH:
			b0 = 1.0f;
			b1 = -2.0f * cs;
			b2 = 1.0f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * cs;
			a2 = 1.0f - alpha;
			break;
	}
	//don't let these states be used until they're updated
	__disable_irq();
		const float a0r = 1.0f / a0;
    state->a0 = b0 * a0r;
    state->a1 = b1 * a0r;
    state->a2 = b2 * a0r;
    state->a3 = a1 * a0r;
    state->a4 = a2 * a0r;
	__enable_irq();
}

#pragma GCC push_options
#pragma GCC optimize ("O3")
float biquad_update(float sample, biquad_axis_state_t *state)
{
    float result;

    /* compute result */
    result = state->a0 * (float)sample + state->a1 * state->x1 + state->a2 * state->x2 -
            state->a3 * state->y1 - state->a4 * state->y2;

	if ( !isnan(result) )
	{
		/* shift x1 to x2, sample to x1 */
		state->x2 = state->x1;
		state->x1 = (float)sample;
		/* shift y1 to y2, result to y1 */
		state->y2 = state->y1;
		state->y1 = result;
	}
	else
	{
		//don't filter is result goes NaN
		return sample;
	}

    return result;

}
#pragma GCC pop_options
