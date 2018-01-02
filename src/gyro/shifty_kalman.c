
#include "shifty_kalman.h"
#include "fix16.h"

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