#include "includes.h"
#include "gyro.h"
#include "gyro_device.h"
#include "imuf_device.h"
#include "imu.h"
#include "board_comm.h"
#include "quaternions.h"
#include "filter.h"

volatile int calibratingGyro;
volatile axisData_t gyroSum;
volatile axisData_t gyroCalibrationTrim;
volatile axisData_t rawAccData;
volatile axisData_t rawRateData;

float gyroTempData;
filteredData_t filteredData;

DMA_InitTypeDef gyroDmaInitStruct;

static void gyro_int_to_float(gyroFrame_t* gyroRxFrame)
{
    static uint32_t gyroCalibrationCycles = 0;
    static int gyroLoopCounter = 0;

    if (gyroLoopCounter-- <= 0)
    {
        //if accDenom is 8, then we should do a switch case for quatenrion math.
        gyroLoopCounter = gyroConfig.accDenom;
        rawAccData.x = ((int16_t)((gyroRxFrame->accelX_H << 8) | gyroRxFrame->accelX_L)) * gyroAccMultiplier;
		rawAccData.y = ((int16_t)((gyroRxFrame->accelY_H << 8) | gyroRxFrame->accelY_L)) * gyroAccMultiplier;
		rawAccData.z = ((int16_t)((gyroRxFrame->accelZ_H << 8) | gyroRxFrame->accelZ_L)) * gyroAccMultiplier;
        gyroTempData = ((int16_t)((gyroRxFrame->temp_H << 8)   | gyroRxFrame->temp_L))   * GYRO_TEMP_MULTIPLIER + 25;
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
    rawRateData.x = (float)((int16_t)((gyroRxFrame->gyroX_H << 8) | gyroRxFrame->gyroX_L)) * gyroRateMultiplier + gyroCalibrationTrim.x;
	rawRateData.y = (float)((int16_t)((gyroRxFrame->gyroY_H << 8) | gyroRxFrame->gyroY_L)) * gyroRateMultiplier + gyroCalibrationTrim.y;
	rawRateData.z = (float)((int16_t)((gyroRxFrame->gyroZ_H << 8) | gyroRxFrame->gyroZ_L)) * gyroRateMultiplier + gyroCalibrationTrim.z;
}

void gyro_read_done(gyroFrame_t* gyroRxFrame) {

    uint32_t accTracker = 8; //start at 7, so 8 is run first
    volatile quaternion_buffer_t *quatBuffer = &(quatBufferA); //start working on this buffer
    //default rateDatafilteredData
    uint8_t* memptr = (uint8_t*)&filteredData.rateData;

    if (boardCommState.commMode == GTBCM_GYRO_ONLY_PASSTHRU) 
    {
        memptr = (uint8_t*)&(gyroRxFrame->gyroX_H);
    } else if (boardCommState.commMode == GTBCM_GYRO_ACC_PASSTHRU) 
    {
        memptr = (uint8_t*)&(gyroRxFrame->accAddress);
    }

    if (boardCommState.commMode == GTBCM_GYRO_ACC_FILTER_F || boardCommState.commMode == GTBCM_GYRO_ONLY_FILTER_F || boardCommState.commMode == GTBCM_GYRO_ACC_QUAT_FILTER_F){
        gyro_int_to_float(gyroRxFrame);
        filter_data(&rawRateData, &rawAccData, gyroTempData, &filteredData); //profile: this takes 2.45us to run with O3 optimization, before adding biquad at least
    }
    
    if (boardCommState.commMode == GTBCM_GYRO_ACC_QUAT_FILTER_F){
        //set flags and do quats in main loop
        //we have to fill the gyro data here though
        //add rate data for later usage in quats. This is reset in imu.c
        quatBuffer->vector.x += filteredData.rateData.x;
        quatBuffer->vector.y += filteredData.rateData.y;
        quatBuffer->vector.z += filteredData.rateData.z;

        accTracker++;
        switch(accTracker)
        {
            case 9:
                //update quaternions, these were calculated in imu.c
                filteredData.quaternion[0] = attitudeFrameQuat.w;
                filteredData.quaternion[1] = attitudeFrameQuat.vector.x;
                filteredData.quaternion[2] = attitudeFrameQuat.vector.y;
                filteredData.quaternion[3] = attitudeFrameQuat.vector.z;
                //put acc into quat buffer
                quatBuffer->accVector.x = filteredData.accData.x;
                quatBuffer->accVector.y = filteredData.accData.y;
                quatBuffer->accVector.z = filteredData.accData.z;
                quatState = QUAT_PROCESS_BUFFER_0;
                //switch buffers
                quatBuffer = &quatBufferB;
                break;
            case 17:
                //reset acc tracker
                accTracker = 1;
                //update quaternions, these were calculated in imu.c
                filteredData.quaternion[0] = attitudeFrameQuat.w;
                filteredData.quaternion[1] = attitudeFrameQuat.vector.x;
                filteredData.quaternion[2] = attitudeFrameQuat.vector.y;
                filteredData.quaternion[3] = attitudeFrameQuat.vector.z;
                //put acc into quat buffer
                quatBuffer->accVector.x = filteredData.accData.x;
                quatBuffer->accVector.y = filteredData.accData.y;
                quatBuffer->accVector.z = filteredData.accData.z;
                quatState = QUAT_PROCESS_BUFFER_1;
                //switch buffers
                quatBuffer = &quatBufferA;
                break;
        }
    }
}

void gyro_init(void) 
{
    gyroTempData = 0;
    calibratingGyro = 0;    
    gyroSum.x = 0.0f;
    gyroSum.y = 0.0f;
    gyroSum.z = 0.0f;
    gyro_device_init(&gyro_read_done);
    imuf_init();
}