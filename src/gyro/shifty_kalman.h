#pragma once
#include <math.h>
#include "fix16.h"

typedef struct shifty_kalman_s {
    fix16_t q;       //process noise covariance
    fix16_t r;       //measurement noise covariance
    fix16_t p;       //estimation error covariance matrix
    fix16_t k;       //kalman gain
    fix16_t x;       //state
    fix16_t lastX;   //previous state
    fix16_t fixed1;   
} shifty_kalman_t;

extern void shifty_kalman_init(void);
extern void shifty_kalman_update(shifty_kalman_t *filter, fix16_t q, fix16_t r, fix16_t p);