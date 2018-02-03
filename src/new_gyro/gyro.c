#include "includes.h"
#include "gyro.h"

volatile int calibratingGyro;
volatile axisData_t gyroSum;
volatile axisData_t gyroCalibrationTrim;
volatile axisData_t rawAccData;
volatile axisData_t rawRateData;

DMA_InitTypeDef gyroDmaInitStruct;

static void gyro_int_to_float(void)
{

    static uint32_t gyroCalibrationCycles = 0;
    static int gyroLoopCounter = 0;

    if (gyroLoopCounter-- <= 0)
    {
        //if accDenom is 8, then we should do a switch case for quatenrion math.
        gyroLoopCounter = gyroConfig.accDenom;
        rawAccData.x = ((int16_t)((gyroRxFrame.accelX_H << 8) | gyroRxFrame.accelX_L)) * gyroAccMultiplier;
		rawAccData.y = ((int16_t)((gyroRxFrame.accelY_H << 8) | gyroRxFrame.accelY_L)) * gyroAccMultiplier;
		rawAccData.z = ((int16_t)((gyroRxFrame.accelZ_H << 8) | gyroRxFrame.accelZ_L)) * gyroAccMultiplier;
        gyroTempData   = ((int16_t)((gyroRxFrame.temp_H << 8)   | gyroRxFrame.temp_L))   * GYRO_TEMP_MULTIPLIER + 25;
        //= (TEMP_OUT[15:0]/Temp_Sensitivity) +
        //RoomTemp_Offset
        //where Temp_Sensitivity = 326.8 LSB/ºC and
        //RoomTemp_Offset = 25ºC
        //gyroTempMultiplier is gyro temp in C
    }

    //doing in real time, might be better to move this to the main loop for processing, but we need to make sure it's done right
    if (calibratingGyro)
    {
        if(gyroCalibrationCycles < CALIBRATION_CYCLES) //limit how many cycles we allow for calibration to minimize float error
        {
            gyroCalibrationCycles++;
            gyroSum.x += rawRateData.x;
            gyroSum.y += rawRateData.y;
            gyroSum.z += rawRateData.z;

        }
        else
        {
            gyroCalibrationTrim.x = -gyroSum.x / (float)gyroCalibrationCycles;
            gyroCalibrationTrim.y = -gyroSum.y / (float)gyroCalibrationCycles;
            gyroCalibrationTrim.z = -gyroSum.z / (float)gyroCalibrationCycles;
            calibratingGyro = 0; //calibration done, set to zero and calibration data will apear in next cycle.
            gyroCalibrationCycles = 0;
        }
    }

    //f*f+f is one operation on FPU
    rawRateData.x = (float)((int16_t)((gyroRxFrame.gyroX_H << 8) | gyroRxFrame.gyroX_L)) * gyroRateMultiplier + gyroCalibrationTrim.x;
	rawRateData.y = (float)((int16_t)((gyroRxFrame.gyroY_H << 8) | gyroRxFrame.gyroY_L)) * gyroRateMultiplier + gyroCalibrationTrim.y;
	rawRateData.z = (float)((int16_t)((gyroRxFrame.gyroZ_H << 8) | gyroRxFrame.gyroZ_L)) * gyroRateMultiplier + gyroCalibrationTrim.z;

}