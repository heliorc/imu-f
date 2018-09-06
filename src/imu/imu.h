#pragma once
#include "quaternions.h"
#include "vectors.h"


//	+X to the right
//	+Y straight up
//	+Z axis toward viewer
//	Heading = rotation about y axis
//	Attitude = rotation about z axis
//	Bank = rotation about x axis

//30 degrees per second
#define MAX_SPIN_RATE_RAD 0.523599f

extern volatile int quadInverted;
extern volatile float currentSpinRate;
extern volatile quaternion_record_t attitudeFrameQuat;
extern volatile float requestedDegrees[3];

extern void init_imu(void);
extern void update_imu(volatile vector_record_t *gyroVector, volatile vector_record_t *accBodyVector, uint32_t step);