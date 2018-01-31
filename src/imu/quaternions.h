#pragma once

#include "includes.h"
#include "vectors.h"

typedef enum quaternionUpdateState
{
    QUAT_NO_DATA          = 0,
    QUAT_PROCESS_BUFFER_0 = 1,
    QUAT_DONE_BUFFER_0    = 2,
    QUAT_PROCESS_BUFFER_1 = 3,
    QUAT_DONE_BUFFER_1    = 4,
} quaternionUpdateState_t;

typedef struct quaternion_buffer {
    vector_record_t vector;
    vector_record_t accVector;
} quaternion_buffer_t;

typedef struct quaternion_record {
    vector_record_t vector;
	volatile float w;
} quaternion_record_t;

extern volatile quaternionUpdateState_t quatState;
extern quaternion_buffer_t *quatBufferA;
extern quaternion_buffer_t *quatBufferB;

extern void MultiplyQuaternionByQuaternion(volatile quaternion_record_t *qOut, volatile quaternion_record_t *q1, volatile quaternion_record_t *q2);
extern void QuaternionNormalize (volatile quaternion_record_t *out);
extern void QuaternionZeroRotation(volatile quaternion_record_t *quaternion);
extern void QuaternionConjugate(volatile quaternion_record_t *out, volatile quaternion_record_t *in);
extern void init_quaternions(void);
extern void update_quaternions(void);