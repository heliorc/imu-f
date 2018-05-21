#include "includes.h"
#include "imu.h"
#include "vectors.h"
#include "quaternions.h"

//#define HALF_GYRO_DT 0.000125f //1/2 of dT for gyro sample rate which is 32 KHz
//#define HALF_GYRO_DT 0.00025f //1/2 of dT for gyro sample rate which is 32 KHz
#define HALF_GYRO_DT 0.000015625f //1/2 of dT for gyro sample rate which is 32 KHz

volatile int quadInverted = 0;
volatile quaternion_record_t gyroQuat;
vector_record_t accWorldVector;
vector_record_t errorWorldVector;
vector_record_t verticalVector;
volatile quaternion_record_t attitudeFrameQuat;
quaternion_record_t conjQuat;
quaternion_record_t multQuat;
quaternion_record_t tempQuat;
volatile float currentSpinRate = 0.0f;
volatile float rotationalMatrix[3][3];

//quats are defined like this: X is roll, Y is pitch, Z is yaw.
//Positive X is a roll to the right which matches our gyro
//Positive Y is a pitch down which is opposite of our gyro
//Positive Z is a yaw to the left which is opposite of our gyro
//we feed the quad negative yaw and pitch values to make it match our gyro

static float Atan2fast( float y, float x )
{
    if ( x == 0.0f )
    {
        if ( y > 0.0f ) return 1.5707963f;
        if ( y == 0.0f ) return 0.0f;
        return -1.5707963f;
    }
    float atan;
    float z = y/x;
    if ( ABS( z ) < 1.0f )
    {
        atan = z/(1.0f + 0.28f*z*z);
        if ( x < 0.0f )
        {
            if ( y < 0.0f ) return atan - 3.14159265f;
            return atan + 3.14159265f;
        }
    }
    else
    {
        atan = 1.5707963f - z/(z*z + 0.28f);
        if ( y < 0.0f ) return atan - 3.14159265f;
    }
    return atan;
}

void init_imu(void)
{
	uint32_t x, y;

	init_quaternions();

	for (x = 0; x < 3; x++)
	{
		for (y = 0; y < 3; y++)
		{
			rotationalMatrix[x][y] = 0.0f;
		}
	}

	VectorZeroVector(&accWorldVector);
	VectorZeroVector(&errorWorldVector);


	//set vertical vector to normallized vertical values
	verticalVector.x = 0.0f;
	verticalVector.y = 0.0f;
	verticalVector.z = 1.0f;

	QuaternionZeroRotation(&conjQuat);
	QuaternionZeroRotation(&multQuat);
	QuaternionZeroRotation(&tempQuat);
	QuaternionZeroRotation(&gyroQuat);
	QuaternionZeroRotation(&attitudeFrameQuat);

    float qxqx = (attitudeFrameQuat.vector.x * attitudeFrameQuat.vector.x);
    float qyqy = (attitudeFrameQuat.vector.y * attitudeFrameQuat.vector.y);
    float qzqz = (attitudeFrameQuat.vector.z * attitudeFrameQuat.vector.z);

    float qwqx = (attitudeFrameQuat.w * attitudeFrameQuat.vector.x);
    float qwqy = (attitudeFrameQuat.w * attitudeFrameQuat.vector.y);
    float qwqz = (attitudeFrameQuat.w * attitudeFrameQuat.vector.z);
    float qxqy = (attitudeFrameQuat.vector.x * attitudeFrameQuat.vector.y);
    float qxqz = (attitudeFrameQuat.vector.x * attitudeFrameQuat.vector.z);
    float qyqz = (attitudeFrameQuat.vector.y * attitudeFrameQuat.vector.z);

    rotationalMatrix[0][0] = (1.0f - 2.0f * qyqy - 2.0f * qzqz);
    rotationalMatrix[0][1] = (2.0f * (qxqy - qwqz));
    rotationalMatrix[0][2] = (2.0f * (qxqz + qwqy));

    rotationalMatrix[1][0] = (2.0f * (qxqy + qwqz));
    rotationalMatrix[1][1] = (1.0f - 2.0f * qxqx - 2.0f * qzqz);
    rotationalMatrix[1][2] = (2.0f * (qyqz - qwqx));

    rotationalMatrix[2][0] = (2.0f * (qxqz - qwqy));
    rotationalMatrix[2][1] = (2.0f * (qyqz + qwqx));
    rotationalMatrix[2][2] = (1.0f - 2.0f * qxqx - 2.0f * qyqy);
}

static void MultiplyQuatAndVector(volatile quaternion_record_t *quatOut, volatile quaternion_record_t *quatIn, volatile vector_record_t *vectorIn)
{
    quatOut->w = -quatIn->vector.x * vectorIn->x - quatIn->vector.y * vectorIn->y - quatIn->vector.z * vectorIn->z;
    quatOut->vector.x =  quatIn->w * vectorIn->x + quatIn->vector.z * vectorIn->y - quatIn->vector.y * vectorIn->z;  
    quatOut->vector.y =  quatIn->w * vectorIn->y + quatIn->vector.x * vectorIn->z - quatIn->vector.z * vectorIn->x;
    quatOut->vector.z =  quatIn->vector.y * vectorIn->x - quatIn->vector.x * vectorIn->y + quatIn->w * vectorIn->z;
}

float get_acc_trust(volatile vector_record_t *gyroVector)
{
    float currentSpinRate;

    //calculate current spin rate in DPS
	arm_sqrt_f32( SQUARE(gyroVector->x) + SQUARE(gyroVector->y) + SQUARE(gyroVector->z), &currentSpinRate);

    //trust ACC a LOT for first 7 seconds
    if(millis() < 4000) //7000 is 7 seconds
    {
        return 1000.0f;
    }
    else
    {
        //vary trust from 2 to 0 based on spin rate (should also look at noise)
        return 100.0f;
        //return CHANGERANGE( CONSTRAIN(currentSpinRate, 0.0f, 500.0f), 500.0f, 0.0f, 0.0f, 50.0f);
    }
}

void normallize_acc(volatile vector_record_t *accBodyVector)
{
	float norm;
    //normallize the ACC readings
    arm_sqrt_f32( (accBodyVector->x * accBodyVector->x + accBodyVector->y * accBodyVector->y + accBodyVector->z * accBodyVector->z), &norm);
    norm = 1.0f/norm;
    accBodyVector->x *= norm;
    accBodyVector->y *= norm;
    accBodyVector->z *= norm;
}

void update_imu(volatile vector_record_t *gyroVector, volatile vector_record_t *accBodyVector, uint32_t step)
{

    switch (step)
    {
        case 0:
            normallize_acc(accBodyVector); //sqrt
        break;
        case 1:
            QuaternionZeroRotation(&conjQuat); //simp
            QuaternionZeroRotation(&multQuat); //simp
            QuaternionZeroRotation(&tempQuat); //simp
            QuaternionConjugate(&conjQuat, &attitudeFrameQuat); //simp
            MultiplyQuatAndVector(&multQuat, &conjQuat, accBodyVector); //complex arithmetic 
        break;
        case 2:
            MultiplyQuaternionByQuaternion(&tempQuat, &multQuat,  &attitudeFrameQuat);
            VectorCrossProduct(&errorWorldVector,  (vector_record_t *)&(tempQuat.vector), &verticalVector);             //find correction error from ACC readings
            QuaternionZeroRotation(&conjQuat);
            QuaternionZeroRotation(&multQuat);
            QuaternionZeroRotation(&tempQuat);
        break;
        case 3:
            MultiplyQuatAndVector(&multQuat, &attitudeFrameQuat, &errorWorldVector);
            QuaternionConjugate(&conjQuat, &attitudeFrameQuat);
            MultiplyQuaternionByQuaternion(&tempQuat, &multQuat, &conjQuat);
        break;
        case 4:
            //apply ACC correction to Gyro Vector with trust modifier
	        VectorAddVector(gyroVector, &(tempQuat.vector), get_acc_trust(gyroVector) );    
        break;
        case 5:
            gyroQuat.vector.x  = DegreesToRadians(gyroVector->x * HALF_GYRO_DT);
            gyroQuat.vector.y  = DegreesToRadians(gyroVector->y * HALF_GYRO_DT);
            gyroQuat.vector.z  = DegreesToRadians(gyroVector->z * HALF_GYRO_DT);
            gyroVector->x = 0;
            gyroVector->y = 0;
            gyroVector->z = 0;
            gyroQuat.w  = 1.0f - 0.5f * ( SQUARE(gyroQuat.vector.x) + SQUARE(gyroQuat.vector.y) + SQUARE(gyroQuat.vector.z) );

            MultiplyQuaternionByQuaternion(&attitudeFrameQuat, &gyroQuat, &attitudeFrameQuat);   //update attitudeFrameQuat quaternion

            if (isnan(attitudeFrameQuat.w) || isnan(attitudeFrameQuat.vector.x) || isnan(attitudeFrameQuat.vector.y) || isnan(attitudeFrameQuat.vector.z))
            {
                attitudeFrameQuat.w = 1.0;
                attitudeFrameQuat.vector.x = 0.0;
                attitudeFrameQuat.vector.y = 0.0;
                attitudeFrameQuat.vector.z = 0.0;
            }
        break;
    }
}
