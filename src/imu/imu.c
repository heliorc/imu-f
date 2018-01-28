#include "includes.h"
#include "imu.h"

#define HALF_GYRO_DT 0.000015625f //1/2 of dT for gyro sample rate which is 32 KHz

volatile quaternionUpdateState_t quatState;
volatile quaternion_buffer_t quatBuffer[2];

volatile int quadInverted = 0;
volatile quaternion_record commandQuat;
volatile quaternion_record accQuat;
volatile quaternion_record gyroQuat;
volatile vector_record gyroVector;
volatile vector_record accBodyVector;
volatile vector_record accWorldVector;
volatile vector_record errorWorldVector;
volatile vector_record errorBodyVector;
volatile vector_record verticalVector;
volatile quaternion_record accBodyQuat;
volatile quaternion_record accWorldQuat;
volatile quaternion_record rotationQuat;
volatile quaternion_record errorQuat;
volatile quaternion_record attitudeFrameQuat;
volatile quaternion_record inertialFrameQuat;

volatile float currentSpinRate = 0.0f;
volatile float rotationalMatrix[3][3];
volatile float requestedDegrees[3];
volatile float rollAttitude;
volatile float pitchAttitude;
volatile float yawAttitude;

static void QuaternionZeroRotation(volatile quaternion_record *quaternion);
static void VectorZeroVector(volatile vector_record *vector);
static void QuaternionNormalize (volatile quaternion_record *out);
static void QuaternionMultiply(volatile quaternion_record *out, volatile quaternion_record *q1, volatile quaternion_record *q2);
static quaternion_record QuaternionFromEuler (float halfBankRads, float halfAttitudeRads, float halfHeadingRads);
//static void QuaternionToEuler(volatile quaternion_record *inQuat, volatile float rates[]);
static void UpdateRotationMatrix(void);
//static void QuaternionMultiplyDps(volatile quaternion_record *quatToUpdate, float gyroRollDiffRads, float gyroPitchDiffRads, float gyroYawDiffRads);
//static void QuaternionDifference(volatile quaternion_record *q1, volatile quaternion_record *q2);

quaternion_record MultiplyQuaternionByQuaternion(volatile quaternion_record q1, volatile quaternion_record q2);
void              GenerateQuaternionFromGyroVector(volatile quaternion_record *quatOut, vector_record vectorIn, float halfdT);
quaternion_record QuaternionConjugate (volatile quaternion_record *out);
void              VectorToQuat(quaternion_record *outQuad, float x, float y, float z);
quaternion_record MultiplyQuatAndVector(volatile quaternion_record quatIn, volatile vector_record vectorIn);
quaternion_record MultiplyQuatAndQuat(volatile quaternion_record quatIn1, volatile quaternion_record quatIn2);
void              EulerToVector(volatile vector_record *outVector, float x, float y, float z);
void              VectorAddVector(volatile vector_record *vectorOut, vector_record vectorIn, float trust);

//quats are defined like this: X is roll, Y is pitch, Z is yaw.
//Positive X is a roll to the right which matches our gyro
//Positive Y is a pitch down which is opposite of our gyro
//Positive Z is a yaw to the left which is opposite of our gyro
//we feed the quad negative yaw and pitch values to make it match our gyro

inline float InlineDegreesToRadians(float degrees)
{
	return(degrees * PI180f);
}

inline float InlineRadiansToDegrees(float radians)
{
	return(radians * d180PIf);
}

inline float Atan2fast( float y, float x )
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

inline float inline_clamp_f(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

inline float inline_change_range_f(float oldValue, float oldMax, float oldMin, float newMax, float newMin)
{
	float oldRange = (oldMax - oldMin);
	float newRange = (newMax - newMin);
	return (((oldValue - oldMin) * newRange) / oldRange) + newMin;
}

static void QuaternionZeroRotation(volatile quaternion_record *quaternion)
{
	quaternion->w = 1.0f;
	quaternion->x = 0.0f;
	quaternion->y = 0.0f;
	quaternion->z = 0.0f;
}

static void VectorZeroVector(volatile vector_record *vector)
{
	vector->x = 0.0f;
	vector->y = 0.0f;
	vector->z = 0.0f;
}

static void QuaternionNormalize (volatile quaternion_record *out)
{
	float norm;
	arm_sqrt_f32( (out->w * out->w + out->x * out->x + out->y * out->y + out->z * out->z), &norm);
	norm = 1.0f/norm;
	out->w *= norm;
	out->x *= norm;
	out->y *= norm;
	out->z *= norm;
}

inline quaternion_record QuaternionConjugate (volatile quaternion_record *out)
{
	quaternion_record outQuat;
	outQuat.w =  out->w;
	outQuat.x = -out->x;
	outQuat.y = -out->y;
	outQuat.z = -out->z;
	return(outQuat);
}

static void QuaternionMultiply (volatile quaternion_record *out, volatile quaternion_record *q1, volatile quaternion_record *q2)
{

	out->x =  q1->x * q2->w + q1->y * q2->z - q1->z * q2->y + q1->w * q2->x;
	out->y = -q1->x * q2->z + q1->y * q2->w + q1->z * q2->x + q1->w * q2->y;
	out->z =  q1->x * q2->y - q1->y * q2->x + q1->z * q2->w + q1->w * q2->z;
	out->w = -q1->x * q2->x - q1->y * q2->y - q1->z * q2->z + q1->w * q2->w;

}

static quaternion_record QuaternionFromEuler (float halfBankRads, float halfAttitudeRads, float halfHeadingRads)
{
	quaternion_record tempQuaternion;
	float c2, c1, c3;
	float s2, s1, s3;

	c1 = arm_cos_f32(halfHeadingRads);
	c2 = arm_cos_f32(halfAttitudeRads);
	c3 = arm_cos_f32(halfBankRads);
	s1 = arm_sin_f32(halfHeadingRads);
	s2 = arm_sin_f32(halfAttitudeRads);
	s3 = arm_sin_f32(halfBankRads);

	tempQuaternion.w = (c1 * c2 * c3 - s1 * s2 * s3);
	tempQuaternion.x = (s1 * s2 * c3 + c1 * c2 * s3);
	tempQuaternion.y = (s1 * c2 * c3 + c1 * s2 * s3);
	tempQuaternion.z = (c1 * s2 * c3 - s1 * c2 * s3);

	return(tempQuaternion);
}

static void UpdateRotationMatrix(void)
{
    float qxqx = (attitudeFrameQuat.x * attitudeFrameQuat.x);
    float qyqy = (attitudeFrameQuat.y * attitudeFrameQuat.y);
    float qzqz = (attitudeFrameQuat.z * attitudeFrameQuat.z);

    float qwqx = (attitudeFrameQuat.w * attitudeFrameQuat.x);
    float qwqy = (attitudeFrameQuat.w * attitudeFrameQuat.y);
    float qwqz = (attitudeFrameQuat.w * attitudeFrameQuat.z);
    float qxqy = (attitudeFrameQuat.x * attitudeFrameQuat.y);
    float qxqz = (attitudeFrameQuat.x * attitudeFrameQuat.z);
    float qyqz = (attitudeFrameQuat.y * attitudeFrameQuat.z);

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

    quatState = QUAT_NO_DATA;

    quatBuffer[0].x = 0.0f;
    quatBuffer[0].y = 0.0f;
    quatBuffer[0].z = 0.0f;
    quatBuffer[0].accx = 0.0f;
    quatBuffer[0].accy = 0.0f;
    quatBuffer[0].accz = 0.0f;

    quatBuffer[1].x = 0.0f;
    quatBuffer[1].y = 0.0f;
    quatBuffer[1].z = 0.0f;
    quatBuffer[1].accx = 0.0f;
    quatBuffer[1].accy = 0.0f;
    quatBuffer[1].accz = 0.0f;

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
	quaternion_record tempQuat;

	tempQuat.w = attitudeFrameQuat.w;
	tempQuat.x = attitudeFrameQuat.x;
	tempQuat.y = attitudeFrameQuat.y;
	tempQuat.z = attitudeFrameQuat.z;

	gyroRollDiffRads  = InlineDegreesToRadians( gyroRollDiffRads )  * HALF_GYRO_DT;
	gyroPitchDiffRads = InlineDegreesToRadians( gyroPitchDiffRads ) * HALF_GYRO_DT;
	gyroYawDiffRads   = InlineDegreesToRadians( gyroYawDiffRads )   * HALF_GYRO_DT;

	if (isnan(gyroRollDiffRads) || isnan(gyroPitchDiffRads) || isnan(gyroYawDiffRads))
	{
		return;
	}

	attitudeFrameQuat.w += (-tempQuat.x * gyroRollDiffRads  - tempQuat.y * gyroPitchDiffRads - tempQuat.z * gyroYawDiffRads);
	attitudeFrameQuat.x += (tempQuat.w  * gyroRollDiffRads  + tempQuat.y * gyroYawDiffRads   - tempQuat.z * gyroPitchDiffRads);
	attitudeFrameQuat.y += (tempQuat.w  * gyroPitchDiffRads - tempQuat.x * gyroYawDiffRads   + tempQuat.z * gyroRollDiffRads);
	attitudeFrameQuat.z += (tempQuat.w  * gyroYawDiffRads   + tempQuat.x * gyroPitchDiffRads - tempQuat.y * gyroRollDiffRads);

	QuaternionNormalize(&attitudeFrameQuat);
}

inline quaternion_record MultiplyQuatAndVector(volatile quaternion_record quatIn, volatile vector_record vectorIn)
{
	quaternion_record quatOut;
    quatOut.w = -quatIn.x * vectorIn.x - quatIn.y * vectorIn.y - quatIn.z * vectorIn.z;
    quatOut.x =  quatIn.w * vectorIn.x + quatIn.z * vectorIn.y - quatIn.y * vectorIn.z;  
    quatOut.y =  quatIn.w * vectorIn.y + quatIn.x * vectorIn.z - quatIn.z * vectorIn.x;
    quatOut.z =  quatIn.y * vectorIn.x - quatIn.x * vectorIn.y + quatIn.w * vectorIn.z;
    return(quatOut);
}

inline quaternion_record MultiplyQuatAndQuat(volatile quaternion_record quatIn1, volatile quaternion_record quatIn2)
{
	quaternion_record quatOut;
    quatOut.x = quatIn1.w * quatIn2.x + quatIn1.z * quatIn2.y - quatIn1.y * quatIn2.z + quatIn1.x * quatIn2.w;  
    quatOut.y = quatIn1.w * quatIn2.y + quatIn1.x * quatIn2.z + quatIn1.y * quatIn2.w - quatIn1.z * quatIn2.x;
    quatOut.z = quatIn1.y * quatIn2.x - quatIn1.x * quatIn2.y + quatIn1.w * quatIn2.z + quatIn1.z * quatIn2.w;
    quatOut.w = quatIn1.w * quatIn2.w - quatIn1.x * quatIn2.x - quatIn1.y * quatIn2.y - quatIn1.z * quatIn2.z;
    return(quatOut);
}

inline vector_record QuaternionToVector(volatile quaternion_record quatIn)
{
    vector_record vectorOut;
    vectorOut.x = quatIn.x;
    vectorOut.y = quatIn.y;
    vectorOut.z = quatIn.z;
    return(vectorOut);
}

inline vector_record RotateVectorByQuaternionQV(volatile quaternion_record quatIn, volatile vector_record vectorIn)
{
	quaternion_record tempQuat;

	tempQuat = MultiplyQuatAndQuat(
		MultiplyQuatAndVector( QuaternionConjugate(&quatIn), vectorIn ),
		quatIn
	);

	return( QuaternionToVector(tempQuat) );
}

inline vector_record RotateVectorByQuaternionVQ(volatile vector_record vectorIn, volatile quaternion_record quatIn)
{
	quaternion_record tempQuat;

	tempQuat = MultiplyQuatAndQuat(
		MultiplyQuatAndVector(quatIn, vectorIn),
		QuaternionConjugate(&quatIn)
	);

	return( QuaternionToVector(tempQuat) );
}

inline void VectorCrossProduct(volatile vector_record *vectorOut, volatile vector_record vectorIn1, volatile vector_record vectorIn2)
{ 
    vectorOut->x = vectorIn1.y * vectorIn2.z - vectorIn1.z * vectorIn2.y;
    vectorOut->y = vectorIn1.z * vectorIn2.x - vectorIn1.x * vectorIn2.z;
    vectorOut->z = vectorIn1.x * vectorIn2.y - vectorIn1.y * vectorIn2.x;
}

inline void EulerToVector(volatile vector_record *outVector, float x, float y, float z)
{
	outVector->x = x;
	outVector->y = y;
	outVector->z = z;
}

inline void VectorAddVector(volatile vector_record *vectorOut, vector_record vectorIn, float trust)
{
    vectorOut->x += vectorIn.x * trust;
    vectorOut->y += vectorIn.y * trust;
    vectorOut->z += vectorIn.z * trust;
}

inline void GenerateQuaternionFromGyroVector(volatile quaternion_record *quatOut, vector_record vectorIn, float halfdT)
{  
    
    quatOut->x  = vectorIn.x * halfdT;
    quatOut->y  = vectorIn.y * halfdT;
    quatOut->z  = vectorIn.z * halfdT;
    quatOut->w  = 1.0f - 0.5f * ( SQUARE(quatOut->x) + SQUARE(quatOut->y) + SQUARE(quatOut->z) );
}

inline quaternion_record MultiplyQuaternionByQuaternion(volatile quaternion_record q1, volatile quaternion_record q2)
{
	quaternion_record returnQuat;
    returnQuat.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	returnQuat.x = q1.w * q2.x + q1.z * q2.y - q1.y * q2.z + q1.x * q2.w;  
    returnQuat.y = q1.w * q2.y + q1.x * q2.z + q1.y * q2.w - q1.z * q2.x;
    returnQuat.z = q1.y * q2.x - q1.x * q2.y + q1.w * q2.z + q1.z * q2.w;
	return(returnQuat);
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
        accTrust = inline_change_range_f( inline_clamp_f(currentSpinRate, 0.0f, 50.0f), 50.0f, 0.0f, -2.0f, 0.0f) * -2.0f;
    }

	//we use radians
	gyroPitch = InlineDegreesToRadians( gyroPitch );
	gyroRoll  = InlineDegreesToRadians( gyroRoll );
	gyroYaw   = InlineDegreesToRadians( gyroYaw );

	//normallize the ACC readings
	arm_sqrt_f32( (accX * accX + accY * accY + accZ * accZ), &norm);
	norm = 1.0f/norm;
	accX *= norm;
	accY *= norm;
	accZ *= norm;

	//this gives us attitude
	//vector_record commandVector = verticalVector;

	//this block takes 6.23 us to run
	EulerToVector(&gyroVector, gyroRoll, gyroPitch, gyroYaw);                          //set gyro vector from gyro readings
	EulerToVector(&accBodyVector, accX, accY, accZ);                                   //set ACC vector from ACC readings
	accWorldVector = RotateVectorByQuaternionQV(attitudeFrameQuat, accBodyVector);     //rotate acc body frame to world frame using attitudeFrameQuat quatenrion
	VectorCrossProduct(&errorWorldVector, accWorldVector, verticalVector);             //find correction error from ACC readings
	errorBodyVector = RotateVectorByQuaternionVQ(errorWorldVector, attitudeFrameQuat); //rotate error world frame to world body using attitudeFrameQuat quatenrion
	VectorAddVector(&gyroVector, errorBodyVector, accTrust);                           //apply ACC correction to Gyro Vector with trust modifier
	GenerateQuaternionFromGyroVector(&gyroQuat, gyroVector, HALF_GYRO_DT);             //generate gyro quaternion from modified gyro vector
	attitudeFrameQuat = MultiplyQuaternionByQuaternion(gyroQuat, attitudeFrameQuat);   //update attitudeFrameQuat quaternion


    rollAttitude  =  InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.y * attitudeFrameQuat.z + attitudeFrameQuat.w * attitudeFrameQuat.x, 0.5f - (attitudeFrameQuat.x * attitudeFrameQuat.x + attitudeFrameQuat.y * attitudeFrameQuat.y)) );
    pitchAttitude =  InlineRadiansToDegrees( arm_sin_f32(2.0f * (attitudeFrameQuat.x * attitudeFrameQuat.z - attitudeFrameQuat.w * attitudeFrameQuat.y)) );
    yawAttitude   = -InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.x * attitudeFrameQuat.y + attitudeFrameQuat.w * attitudeFrameQuat.z, 0.5f - (attitudeFrameQuat.y * attitudeFrameQuat.y + attitudeFrameQuat.z * attitudeFrameQuat.z)) );

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

void update_quaternions(void)
{
    switch (quatState)
    {
        case QUAT_PROCESS_BUFFER_0:
            update_imu(quatBuffer[0].x, quatBuffer[0].y, quatBuffer[0].z, quatBuffer[0].accx, quatBuffer[0].accy, quatBuffer[0].accz);
            quatState = QUAT_DONE_BUFFER_0;
            quatBuffer[0].x = 0.0f;
            quatBuffer[0].y = 0.0f;
            quatBuffer[0].z = 0.0f;
            break;
        case QUAT_PROCESS_BUFFER_1:
            update_imu(quatBuffer[1].x, quatBuffer[1].y, quatBuffer[1].z, quatBuffer[1].accx, quatBuffer[1].accy, quatBuffer[1].accz);
            quatState = QUAT_DONE_BUFFER_1;
            quatBuffer[1].x = 0.0f;
            quatBuffer[1].y = 0.0f;
            quatBuffer[1].z = 0.0f;
            break;
        case QUAT_NO_DATA:
        case QUAT_DONE_BUFFER_0:
        case QUAT_DONE_BUFFER_1:
        default:
        break;
    }
}