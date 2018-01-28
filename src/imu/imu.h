#pragma once

//	+X to the right
//	+Y straight up
//	+Z axis toward viewer
//	Heading = rotation about y axis
//	Attitude = rotation about z axis
//	Bank = rotation about x axis

//30 degrees per second
#define MAX_SPIN_RATE_RAD 0.523599f

#define PIf       3.14159265358f
#define HALF_PI_F 1.57079632679f
#define IPIf      0.31830988618f //inverse of Pi
#define I180      0.00555555555f
#define PI180f    0.01745329251f
#define d180PIf   57.2957795131f

#ifndef SQUARE
  #define SQUARE(x) ((x)*(x))
#endif

//These were defined in a lib file?
#ifndef MIN
  #define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
  #define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef ABS
  #define ABS(x) ((x) > 0 ? (x) : -(x))
#endif

#ifndef CONSTRAIN
  #define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif


typedef enum quaternionUpdateState
{
    QUAT_NO_DATA          = 0,
    QUAT_PROCESS_BUFFER_0 = 1,
    QUAT_PROCESS_BUFFER_1 = 2,
    QUAT_DONE_BUFFER_0    = 3,
    QUAT_DONE_BUFFER_1    = 4,
} quaternionUpdateState_t;

typedef struct quaternion_buffer {
	volatile float x;
	volatile float y;
	volatile float z;
	volatile float accx;
	volatile float accy;
	volatile float accz;
} quaternion_buffer_t;

typedef struct {
	volatile float x;
	volatile float y;
	volatile float z;
	volatile float w;
} quaternion_record;

typedef struct {
	volatile float x;
	volatile float y;
	volatile float z;
} vector_record;

extern volatile float rollAttitude;
extern volatile float pitchAttitude;
extern volatile float yawAttitude;

extern volatile quaternion_buffer_t quatBuffer[];
extern volatile quaternionUpdateState_t quatState;
extern volatile int quadInverted;
extern volatile float currentSpinRate;
extern volatile quaternion_record attitudeFrameQuat;
extern volatile float requestedDegrees[3];

extern void update_quaternions();
extern void init_imu(void);
extern void update_imu(float gx, float gy, float gz, float ax, float ay, float az);