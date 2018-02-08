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
volatile float rollAttitude;
volatile float pitchAttitude;
volatile float yawAttitude;

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

    rollAttitude  = 0.0f;
    pitchAttitude = 0.0f;
    yawAttitude   = 0.0f;
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

void update_imu(volatile vector_record_t *gyroVector, volatile vector_record_t *accBodyVector)
{
	volatile float accTrust = 1000.0f;
	float norm;
    static uint32_t forcedUntilBelow = 0;
	static uint32_t forcedUntilAbove = 0;

    //calculate current spin rate in DPS
	arm_sqrt_f32( SQUARE(gyroVector->x) + SQUARE(gyroVector->y) + SQUARE(gyroVector->z), &norm);
	currentSpinRate = norm;
    //trust ACC a LOT for first 7 seconds
    if(millis() < 7000) //7000 is 7 seconds
    {
        accTrust = 10000.0f;
    }
    else
    {
        //vary trust from 2 to 0 based on spin rate (should also look at noise)
        //accTrust = CHANGERANGE( CONSTRAIN(currentSpinRate, 0.0f, 360.0f), 50.0f, 0.0f, -2.0f, 0.0f) * -2.0f;
        accTrust = CHANGERANGE( CONSTRAIN(currentSpinRate, 0.0f, 500.0f), 500.0f, 0.0f, 0.0f, 50.0f);
    }

	//accTrust = 100.0f;
	//we use radians
	//gyroVector->x = 0; //DegreesToRadians( gyroVector->x );
	//gyroVector->y = 0; //DegreesToRadians( gyroVector->y );
	//gyroVector->z = 0; //DegreesToRadians( gyroVector->z );

	//normallize the ACC readings
	arm_sqrt_f32( (accBodyVector->x * accBodyVector->x + accBodyVector->y * accBodyVector->y + accBodyVector->z * accBodyVector->z), &norm);
	norm = 1.0f/norm;
	accBodyVector->x *= norm;
	accBodyVector->y *= norm;
	accBodyVector->z *= norm;

	//this gives us attitude
	//vector_record_t commandVector = verticalVector;

	//this block takes 6.23 us to run
	// RotateVectorByQuaternionQV(&accWorldVector, &attitudeFrameQuat, &accBodyVector);     //rotate acc body frame to world frame using attitudeFrameQuat quatenrion
	QuaternionZeroRotation(&conjQuat);
	QuaternionZeroRotation(&multQuat);
	QuaternionZeroRotation(&tempQuat);
	QuaternionConjugate(&conjQuat, &attitudeFrameQuat);
	MultiplyQuatAndVector(&multQuat, &conjQuat, accBodyVector);
	MultiplyQuaternionByQuaternion(&tempQuat, &multQuat,  &attitudeFrameQuat);
	VectorCrossProduct(&errorWorldVector,  (vector_record_t *)&(tempQuat.vector), &verticalVector);             //find correction error from ACC readings
	QuaternionZeroRotation(&conjQuat);
	QuaternionZeroRotation(&multQuat);
	QuaternionZeroRotation(&tempQuat);
	MultiplyQuatAndVector(&multQuat, &attitudeFrameQuat, &errorWorldVector);
	QuaternionConjugate(&conjQuat, &attitudeFrameQuat);
	MultiplyQuaternionByQuaternion(&tempQuat, &multQuat, &conjQuat);
	VectorAddVector(gyroVector, &(tempQuat.vector), accTrust);                           //apply ACC correction to Gyro Vector with trust modifier

	//this is wrong, but it works?
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

    rollAttitude  =  RadiansToDegrees( Atan2fast(attitudeFrameQuat.vector.y * attitudeFrameQuat.vector.z + attitudeFrameQuat.w * attitudeFrameQuat.vector.x, 0.5f - (attitudeFrameQuat.vector.x * attitudeFrameQuat.vector.x + attitudeFrameQuat.vector.y * attitudeFrameQuat.vector.y)) );
    pitchAttitude =  RadiansToDegrees( arm_sin_f32(2.0f * (attitudeFrameQuat.vector.x * attitudeFrameQuat.vector.z - attitudeFrameQuat.w * attitudeFrameQuat.vector.y)) );
    yawAttitude   = -RadiansToDegrees( Atan2fast(attitudeFrameQuat.vector.x * attitudeFrameQuat.vector.y + attitudeFrameQuat.w * attitudeFrameQuat.vector.z, 0.5f - (attitudeFrameQuat.vector.y * attitudeFrameQuat.vector.y + attitudeFrameQuat.vector.z * attitudeFrameQuat.vector.z)) );

    if( (rollAttitude < -100) || (rollAttitude > 100) )
    {
    	quadInverted = 1;
    }
	else
    {
		quadInverted = 0;
    }


    //this is super hacky
	if ( (forcedUntilBelow) || (pitchAttitude > 88.0f) )
	{
		forcedUntilBelow = 1;
		if ( (pitchAttitude < 88.0f) && (pitchAttitude > 1.0f) )
		{
			forcedUntilBelow = 0;
		}
		else
		{
			pitchAttitude = 90.0f;
		}
	}

    //this is super hacky
	if ( (forcedUntilAbove) || (pitchAttitude < -88.0f) )
	{
		forcedUntilAbove = 1;
		if ( (pitchAttitude > -88.0f) && (pitchAttitude < -1.0f) )
		{
			forcedUntilAbove = 0;
		}
		else
		{
			pitchAttitude = -90.0f;
		}
	}
}
