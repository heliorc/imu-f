#include "includes.h"
#include "imu.h"
#include "vectors.h"
#include "quaternions.h"

#define HALF_GYRO_DT 0.000015625f //1/2 of dT for gyro sample rate which is 32 KHz

volatile int quadInverted = 0;
volatile quaternion_record_t commandQuat;
volatile quaternion_record_t accQuat;
volatile quaternion_record_t gyroQuat;
vector_record_t gyroVector;
vector_record_t accBodyVector;
vector_record_t accWorldVector;
vector_record_t errorWorldVector;
vector_record_t errorBodyVector;
vector_record_t verticalVector;
volatile quaternion_record_t accBodyQuat;
volatile quaternion_record_t accWorldQuat;
volatile quaternion_record_t rotationQuat;
volatile quaternion_record_t errorQuat;
volatile quaternion_record_t attitudeFrameQuat;
volatile quaternion_record_t inertialFrameQuat;

volatile float currentSpinRate = 0.0f;
volatile float rotationalMatrix[3][3];
volatile float requestedDegrees[3];
volatile float rollAttitude;
volatile float pitchAttitude;
volatile float yawAttitude;

static void UpdateRotationMatrix(void);

static void GenerateQuaternionFromGyroVector(volatile quaternion_record_t *quatOut, vector_record_t *vectorIn, float halfdT)
{  
    quatOut->vector.x  = vectorIn->x * halfdT;
    quatOut->vector.y  = vectorIn->y * halfdT;
    quatOut->vector.z  = vectorIn->z * halfdT;
    quatOut->w  = 1.0f - 0.5f * ( SQUARE(quatOut->vector.x) + SQUARE(quatOut->vector.y) + SQUARE(quatOut->vector.z) );
}
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

static void UpdateRotationMatrix(void)
{
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

	VectorZeroVector(&gyroVector);
	VectorZeroVector(&accBodyVector);
	VectorZeroVector(&accWorldVector);
	VectorZeroVector(&errorWorldVector);
	VectorZeroVector(&errorBodyVector);

	//set vertical vector to normallized vertical values
	verticalVector.x = 0.0f;
	verticalVector.y = 0.0f;
	verticalVector.z = 1.0f;

	QuaternionZeroRotation(&gyroQuat);
	QuaternionZeroRotation(&attitudeFrameQuat);
	QuaternionZeroRotation(&inertialFrameQuat);
	QuaternionZeroRotation(&rotationQuat);
	QuaternionZeroRotation(&commandQuat);

	UpdateRotationMatrix();
	requestedDegrees[0] = 0.0f;
	requestedDegrees[1] = 0.0f;
	requestedDegrees[2] = 0.0f;
}

void UpdateAttitudeFrameQuat(float gyroRollDiffRads, float gyroPitchDiffRads, float gyroYawDiffRads)
{
	quaternion_record_t tempQuat;

	tempQuat.w = attitudeFrameQuat.w;
	tempQuat.vector.x = attitudeFrameQuat.vector.x;
	tempQuat.vector.y = attitudeFrameQuat.vector.y;
	tempQuat.vector.z = attitudeFrameQuat.vector.z;

	gyroRollDiffRads  = DegreesToRadians( gyroRollDiffRads )  * HALF_GYRO_DT;
	gyroPitchDiffRads = DegreesToRadians( gyroPitchDiffRads ) * HALF_GYRO_DT;
	gyroYawDiffRads   = DegreesToRadians( gyroYawDiffRads )   * HALF_GYRO_DT;

	if (isnan(gyroRollDiffRads) || isnan(gyroPitchDiffRads) || isnan(gyroYawDiffRads))
	{
		return;
	}

	attitudeFrameQuat.w += (-tempQuat.vector.x * gyroRollDiffRads  - tempQuat.vector.y * gyroPitchDiffRads - tempQuat.vector.z * gyroYawDiffRads);
	attitudeFrameQuat.vector.x += (tempQuat.w  * gyroRollDiffRads  + tempQuat.vector.y * gyroYawDiffRads   - tempQuat.vector.z * gyroPitchDiffRads);
	attitudeFrameQuat.vector.y += (tempQuat.w  * gyroPitchDiffRads - tempQuat.vector.x * gyroYawDiffRads   + tempQuat.vector.z * gyroRollDiffRads);
	attitudeFrameQuat.vector.z += (tempQuat.w  * gyroYawDiffRads   + tempQuat.vector.x * gyroPitchDiffRads - tempQuat.vector.y * gyroRollDiffRads);

	QuaternionNormalize(&attitudeFrameQuat);
}

static void MultiplyQuatAndVector(volatile quaternion_record_t *quatOut, volatile quaternion_record_t *quatIn, volatile vector_record_t *vectorIn)
{
    quatOut->w = -quatIn->vector.x * vectorIn->x - quatIn->vector.y * vectorIn->y - quatIn->vector.z * vectorIn->z;
    quatOut->vector.x =  quatIn->w * vectorIn->x + quatIn->vector.z * vectorIn->y - quatIn->vector.y * vectorIn->z;  
    quatOut->vector.y =  quatIn->w * vectorIn->y + quatIn->vector.x * vectorIn->z - quatIn->vector.z * vectorIn->x;
    quatOut->vector.z =  quatIn->vector.y * vectorIn->x - quatIn->vector.x * vectorIn->y + quatIn->w * vectorIn->z;
}

static void RotateVectorByQuaternionQV(volatile vector_record_t *vectorOut, volatile quaternion_record_t *quatIn, volatile vector_record_t *vectorIn)
{
	quaternion_record_t conjQuat;
	QuaternionConjugate(&conjQuat, quatIn);
	quaternion_record_t multQuat;
	MultiplyQuatAndVector(&multQuat, &conjQuat, vectorIn);
	quaternion_record_t tempQuat;
	MultiplyQuaternionByQuaternion(&tempQuat, &multQuat, quatIn);
	vectorOut = &(tempQuat.vector);
}

static void RotateVectorByQuaternionVQ(volatile vector_record_t *vectorOut, volatile vector_record_t *vectorIn, volatile quaternion_record_t *quatIn)
{
	quaternion_record_t multQuat;
	MultiplyQuatAndVector(&multQuat, quatIn, vectorIn);
	quaternion_record_t conjQuat;
	QuaternionConjugate(&conjQuat, quatIn);
	quaternion_record_t tempQuat;	
	MultiplyQuaternionByQuaternion(&tempQuat, &multQuat, &conjQuat);
	vectorOut = &(tempQuat.vector);
}

void update_imu(float accX, float accY, float accZ, float gyroRoll, float gyroPitch, float gyroYaw)
{

	float accTrust = 0.01;
	float norm;
	float accToGyroError[3] = {0.0f, 0.0f, 0.0f};
    static int interationCounter = 0;
	static float accTrustKiStorage[3] = {0.0f, 0.0f, 0.0f};
    static uint32_t forcedUntilBelow = 0;
	static uint32_t forcedUntilAbove = 0;

	if (isnan(accX) || isnan(accY) || isnan(accZ) || isnan(gyroRoll) || isnan(gyroPitch) || isnan(gyroYaw))
	{
		return;
	}

    //calculate current spin rate in DPS
	arm_sqrt_f32( SQUARE(gyroRoll) + SQUARE(gyroPitch) + SQUARE(gyroYaw), &norm);
	currentSpinRate = norm;

    //trust ACC a LOT for first 7 seconds
    if(interationCounter < 7000) //7000 is 7 seconds
    {
        accTrust = 100.0f;
        interationCounter++;
    }
    else
    {
        //vary trust from 2 to 0 based on spin rate (should also look at noise)
        accTrust = CHANGERANGE( CONSTRAIN(currentSpinRate, 0.0f, 50.0f), 50.0f, 0.0f, -2.0f, 0.0f) * -2.0f;
    }

	//we use radians
	gyroPitch = DegreesToRadians( gyroPitch );
	gyroRoll  = DegreesToRadians( gyroRoll );
	gyroYaw   = DegreesToRadians( gyroYaw );

	//normallize the ACC readings
	arm_sqrt_f32( (accX * accX + accY * accY + accZ * accZ), &norm);
	norm = 1.0f/norm;
	accX *= norm;
	accY *= norm;
	accZ *= norm;

	//this gives us attitude
	//vector_record_t commandVector = verticalVector;

	//this block takes 6.23 us to run
	gyroVector.x = gyroRoll;
	gyroVector.y = gyroPitch;
	gyroVector.z = gyroYaw;
	accBodyVector.x = accX;
	accBodyVector.y = accY;
	accBodyVector.z = accZ;                                //set ACC vector from ACC readings
	RotateVectorByQuaternionQV(&accWorldVector, &attitudeFrameQuat, &accBodyVector);     //rotate acc body frame to world frame using attitudeFrameQuat quatenrion
	VectorCrossProduct(&errorWorldVector, &accWorldVector, &verticalVector);             //find correction error from ACC readings
	RotateVectorByQuaternionVQ(&errorBodyVector, &errorWorldVector, &attitudeFrameQuat); //rotate error world frame to world body using attitudeFrameQuat quatenrion
	VectorAddVector(&gyroVector, &errorBodyVector, accTrust);                           //apply ACC correction to Gyro Vector with trust modifier
	GenerateQuaternionFromGyroVector(&gyroQuat, &gyroVector, HALF_GYRO_DT);          //generate gyro quaternion from modified gyro vector
	MultiplyQuaternionByQuaternion(&attitudeFrameQuat, &gyroQuat, &attitudeFrameQuat);   //update attitudeFrameQuat quaternion

	if (isnan(attitudeFrameQuat.w) || isnan(attitudeFrameQuat.vector.x) || isnan(attitudeFrameQuat.vector.y) || isnan(attitudeFrameQuat.vector.z))
	{
		attitudeFrameQuat.w = 1.0;
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