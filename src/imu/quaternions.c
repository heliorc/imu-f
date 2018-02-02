#include "quaternions.h"
#include "includes.h"
#include "gyro.h"
#include "imu.h"

volatile quaternionUpdateState_t quatState;
quaternion_buffer_t quatBufferA;
quaternion_buffer_t quatBufferB;

void buffer_init(quaternion_buffer_t *buffer) {
   buffer->vector.x = 0.0f;
   buffer->vector.y = 0.0f;
   buffer->vector.z = 0.0f;
   buffer->accVector.x = 0.0f;
   buffer->accVector.y = 0.0f;
   buffer->accVector.z = 0.0f;
}

void init_quaternions(void){
   quatState = QUAT_NO_DATA;
   buffer_init(&quatBufferA);
   buffer_init(&quatBufferB);
}

void process(quaternion_buffer_t *quatBuffer) {
    if (isnan(quatBuffer->vector.x) || 
        isnan(quatBuffer->vector.y) || 
        isnan(quatBuffer->vector.z) || 
        isnan(quatBuffer->accVector.x) || 
        isnan(quatBuffer->accVector.y) || 
        isnan(quatBuffer->accVector.z)) {
		return;
	}
    update_imu(&(quatBuffer->vector), &(quatBuffer->accVector));
    quatBuffer->vector.x = 0.0f;
    quatBuffer->vector.y = 0.0f;
    quatBuffer->vector.z = 0.0f;
}

void update_quaternions(void)
{
    switch (quatState)
    {
        case QUAT_PROCESS_BUFFER_0:
            process(&quatBufferA);
            quatState = QUAT_DONE_BUFFER_0;
            break;
        case QUAT_PROCESS_BUFFER_1:        
            process(&quatBufferB);
            quatState = QUAT_DONE_BUFFER_1;      
            break;
        case QUAT_NO_DATA:
        case QUAT_DONE_BUFFER_0:
        case QUAT_DONE_BUFFER_1:
        default:
        break;
    }
}


void QuaternionConjugate(volatile quaternion_record_t *out, volatile quaternion_record_t *in)
{
	out->w =  in->w;
	out->vector.x = -in->vector.x;
	out->vector.y = -in->vector.y;
	out->vector.z = -in->vector.z;
}

void QuaternionZeroRotation(volatile quaternion_record_t *quaternion)
{
	quaternion->w = 1.0f;
	quaternion->vector.x = 0.0f;
	quaternion->vector.y = 0.0f;
	quaternion->vector.z = 0.0f;
}

void MultiplyQuaternionByQuaternion(volatile quaternion_record_t *qOut, volatile quaternion_record_t *q1, volatile quaternion_record_t *q2)
{
    qOut->w = (q1->w * q2->w) - (q1->vector.x * q2->vector.x) - (q1->vector.y * q2->vector.y) - (q1->vector.z * q2->vector.z);
	qOut->vector.x = (q1->w * q2->vector.x) + (q1->vector.x * q2->w) + (q1->vector.y * q2->vector.z) - (q1->vector.z * q2->vector.y);  
    qOut->vector.y = (q1->w * q2->vector.y) - (q1->vector.x * q2->vector.z) + (q1->vector.y * q2->w) + (q1->vector.z * q2->vector.x);
    qOut->vector.z = (q1->w * q2->vector.z) + (q1->vector.x * q2->vector.y) - (q1->vector.y * q2->vector.x) + (q1->vector.z * q2->w);
}


void QuaternionNormalize (volatile quaternion_record_t *out)
{
	float norm;
	arm_sqrt_f32( (out->w * out->w + out->vector.x * out->vector.x + out->vector.y * out->vector.y + out->vector.z * out->vector.z), &norm);
	norm = 1.0f/norm;
	out->w *= norm;
	out->vector.x *= norm;
	out->vector.y *= norm;
	out->vector.z *= norm;
}