
#include "shifty_kalman.h"
#include "fix16.h"




//Fast two-state Kalman
void fastKalmanInitF(fastKalmanF_t *filter, float q, float r, float p, float intialValue)
{
	filter->q     = q * 0.001f; //add multiplier to make tuning easier
	filter->r     = r;    //add multiplier to make tuning easier
	filter->p     = p;    //add multiplier to make tuning easier
	filter->x     = intialValue;   //set intial value, can be zero if unknown
	filter->lastX = intialValue;   //set intial value, can be zero if unknown
	filter->k     = 0.0f;          //kalman gain,  
}

float fastKalmanUpdateF(fastKalmanF_t *filter, float input)
{

    //project the state ahead using acceleration
    filter->x += (filter->x - filter->lastX);

    //update last state
    filter->lastX = filter->x;

	//prediction update
	filter->p = filter->p + filter->q;

	//measurement update
	filter->k = filter->p / (filter->p + filter->r);
	filter->x += filter->k * (input - filter->x);
	filter->p = (1.0f - filter->k) * filter->p;

    //printf("k: %f, p: %f, q: %f, x: %f\n", filter->k, filter->p, filter->q, (filter->x) );

    return filter->x;
}



//SHIFTY_KALMAN
void shifty_kalman_init(shifty_kalman_t *filter, fix16_t q, fix16_t r, fix16_t p)
{
	filter->q     = q;
	filter->r     = r;
	filter->p     = p;
	filter->x     = fix16_from_int(0);   //set intial value, can be zero if unknown
	filter->lastX = fix16_from_int(0);   //set intial value, can be zero if unknown
	filter->k     = fix16_from_int(0);   //kalman gain, will be between 0-1
	filter->fixed1= fix16_from_int(1);   
}

fix16_t shifty_kalman_update(shifty_kalman_t *filter, fix16_t input)
{
    //project the state ahead using acceleration
    filter->x += fix16_sub(filter->x, filter->lastX);
    //update last state
    filter->lastX = filter->x;
	//prediction update
	filter->p = fix16_add(filter->p, filter->q);
	//measurement update
	filter->k = fix16_div(filter->p, fix16_add(filter->p, filter->r));
	filter->x += fix16_mul(filter->k, fix16_sub(input, filter->x));
	filter->p = fix16_mul(fix16_sub(filter->fixed1, filter->k), (filter->p));
    return filter->x;
}