#pragma once

#include "includes.h"
#include "vectors.h"

typedef enum quaternionUpdateState
{
    QUAT_NO_DATA            = 0,
    QUAT_PROCESS_BUFFER_0_0 = 1,
    QUAT_PROCESS_BUFFER_0_1 = 2,
    QUAT_PROCESS_BUFFER_0_2 = 3,
    QUAT_PROCESS_BUFFER_0_3 = 4,
    QUAT_PROCESS_BUFFER_0_4 = 5,
    QUAT_PROCESS_BUFFER_0_5 = 6,
    QUAT_DONE_BUFFER_0      = 7,
    QUAT_PROCESS_BUFFER_1_0 = 8,
    QUAT_PROCESS_BUFFER_1_1 = 9,
    QUAT_PROCESS_BUFFER_1_2 = 10,
    QUAT_PROCESS_BUFFER_1_3 = 11,
    QUAT_PROCESS_BUFFER_1_4 = 12,
    QUAT_PROCESS_BUFFER_1_5 = 13,
    QUAT_DONE_BUFFER_1      = 14,
} quaternionUpdateState_t;

typedef struct quaternion_buffer {
    vector_record_t vector;
    vector_record_t accVector;
}  __attribute__((__packed__)) quaternion_buffer_t;

typedef struct quaternion_record {
	volatile float w;
	volatile vector_record_t vector;
}  __attribute__((__packed__)) quaternion_record_t;

extern volatile quaternionUpdateState_t quatState;
extern volatile quaternion_buffer_t quatBufferA;
extern volatile quaternion_buffer_t quatBufferB;

extern void MultiplyQuaternionByQuaternion(volatile quaternion_record_t *qOut, volatile quaternion_record_t *q1, volatile quaternion_record_t *q2);
extern void QuaternionNormalize (volatile quaternion_record_t *out);
extern void QuaternionZeroRotation(volatile quaternion_record_t *quaternion);
extern void QuaternionConjugate(volatile quaternion_record_t *out, volatile quaternion_record_t *in);
extern void init_quaternions(void);
extern void update_quaternions(void);