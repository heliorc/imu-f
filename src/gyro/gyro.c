#include "includes.h"
#include "gyro.h"
#include "gyro_device.h"
#include "imu.h"
#include "board_comm.h"
#include "quaternions.h"
#include "filter.h"
#include "crc.h"
#include "fft.h"

volatile int calibratingGyro;
volatile axisData_t gyroSum;
volatile axisData_t gyroCalibrationTrim;
volatile axisData_t rawAccData;
volatile axisData_t rawRateData;
volatile gyro_settings_config_t gyroSettingsConfig;

float gyroTempData;
filteredData_t filteredData;

DMA_InitTypeDef gyroDmaInitStruct;

#pragma GCC push_options
#pragma GCC optimize ("Os")

enum
{
    CW0       = 0,
    CW90      = 1,
    CW180     = 2,
    CW270     = 3,
    CW0_INV   = 4,
    CW90_INV  = 5,
    CW180_INV = 6,
    CW270_INV = 7,
    CW45      = 8,
    CW135     = 9,
    CW225     = 10,
    CW315     = 11,
    CW45_INV  = 12,
    CW135_INV = 13,
    CW225_INV = 14,
    CW315_INV = 15,
};

static float rotationMatrix[3][3];
static int   matrixFormed = -1;

//build the rotational matrix for allowing different board angles other than 90 degrees
static void build_rotation_matrix(int x, int y, int z)
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;
    float xRadians, yRadians, zRadians;

    zRadians = ((float)z * PIf * I180);
    yRadians = ((float)y * PIf * I180);
    xRadians = ((float)x * PIf * I180);

    cosz = cosf(zRadians);
    sinz = sinf(zRadians);
    cosy = cosf(yRadians);
    siny = sinf(yRadians);
    cosx = cosf(xRadians);
    sinx = sinf(xRadians);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    rotationMatrix[0][0] = cosz * cosy;
    rotationMatrix[0][1] = -cosy * sinz;
    rotationMatrix[0][2] = siny;
    rotationMatrix[1][0] = sinzcosx + (coszsinx * siny);
    rotationMatrix[1][1] = coszcosx - (sinzsinx * siny);
    rotationMatrix[1][2] = -sinx * cosy;
    rotationMatrix[2][0] = (sinzsinx) - (coszcosx * siny);
    rotationMatrix[2][1] = (coszsinx) + (sinzcosx * siny);
    rotationMatrix[2][2] = cosy * cosx;
}

//force rebuilding of rotational matrix
void reset_matrix(void)
{
	matrixFormed =-1;
}

//int defaults for oreintation
static void init_orientation(void)
{
    //set defaults
    reset_matrix();
    gyroSettingsConfig.orientation = CW0;
    gyroSettingsConfig.smallX      = 0;
    gyroSettingsConfig.smallY      = 0;
    gyroSettingsConfig.smallZ      = 0;
}
#pragma GCC pop_options

#pragma GCC push_options
#pragma GCC optimize ("O3")
//set board orientation
static void apply_gyro_acc_rotation(volatile axisData_t* rawData)
{

	uint32_t nonNinety;
	//from gyro, x, y, z (0, 1, 2)
	// x is roll, y is pitch, z is yaw

    float fx = rawData->x;
    float fy = rawData->y;
    float fz = rawData->z;
	int32_t x = -(gyroSettingsConfig.smallX); //do we invert here?
	int32_t y = (gyroSettingsConfig.smallY);
	int32_t z = (gyroSettingsConfig.smallZ);
	nonNinety = 0;
    switch (gyroSettingsConfig.orientation)
    {

		case CW0:
	    	if (x || y || z)
	    	{
				if (matrixFormed != CW0) {
					matrixFormed = CW0;
					build_rotation_matrix(x,y,z); //x, y, z, pitch, roll, yaw
				}
				nonNinety = 1;
	    	}
	    	else
	    	{
				rawData->x = (fx);
				rawData->y = (fy);
				rawData->z = (fz);
	    	}
			break;
        case CW90:
        	rawData->x = (fy);
        	rawData->y = -(fx);
        	rawData->z = (fz);
            break;
        case CW180:
        	rawData->x = -(fx);
        	rawData->y = -(fy);
        	rawData->z = (fz);
            break;
        case CW270:
        	rawData->x = -(fy);
        	rawData->y = (fx);
        	rawData->z = (fz);
            break;
        case CW0_INV:
        	rawData->x = -(fx);
        	rawData->y = (fy);
        	rawData->z = -(fz);
            break;
        case CW90_INV:
        	rawData->x = (fy);
        	rawData->y = (fx);
        	rawData->z = -(fz);
            break;
        case CW180_INV:
        	rawData->x = (fx);
        	rawData->y = -(fy);
        	rawData->z = -(fz);
            break;
        case CW270_INV:
        	rawData->x = -(fy);
        	rawData->y = -(fx);
        	rawData->z = -(fz);
            break;
    	case CW45:
    		if (matrixFormed != CW45) {
    			matrixFormed = CW45;
    			build_rotation_matrix(0,0,45); //x, y, z, pitch, roll, yaw
    		}
    		nonNinety = 1;
       		break;
    	case CW135:
    		if (matrixFormed != CW135) {
				matrixFormed = CW135;
				build_rotation_matrix(0,0,135); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW225:
    		if (matrixFormed != CW225) {
				matrixFormed = CW225;
				build_rotation_matrix(0,0,225); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW315:
    		if (matrixFormed != CW315) {
				matrixFormed = CW315;
				build_rotation_matrix(0,0,315); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW45_INV:
    		if (matrixFormed != CW45_INV) {
				matrixFormed = CW45_INV;
				build_rotation_matrix(180,0,45); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW135_INV:
    		if (matrixFormed != CW135_INV) {
				matrixFormed = CW135_INV;
				build_rotation_matrix(180,0,135); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW225_INV:
    		if (matrixFormed != CW225_INV) {
				matrixFormed = CW225_INV;
				build_rotation_matrix(180,0,225); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW315_INV:
    		if (matrixFormed != CW315_INV) {
				matrixFormed = CW315_INV;
				build_rotation_matrix(180,0,315); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    }

    if (nonNinety)
    {
		rawData->x = (rotationMatrix[0][0] * fx + rotationMatrix[1][0] * fy + rotationMatrix[2][0] * fz);
		rawData->y = (rotationMatrix[0][1] * fx + rotationMatrix[1][1] * fy + rotationMatrix[2][1] * fz);
		rawData->z = (rotationMatrix[0][2] * fx + rotationMatrix[1][2] * fy + rotationMatrix[2][2] * fz);
    }

}
#pragma GCC pop_options

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
        apply_gyro_acc_rotation(&rawAccData);
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
    apply_gyro_acc_rotation(&rawRateData);
}

void gyro_read_done(gyroFrame_t* gyroRxFrame) {

    static uint32_t accTracker = 8; //start at 7, so 8 is run first
    static volatile quaternion_buffer_t *quatBuffer = &(quatBufferA); //start working on this buffer
    //default rateDatafilteredData
    uint8_t* memptr = (uint8_t*)&filteredData.rateData;

    if (boardCommState.commMode == GTBCM_GYRO_ONLY_PASSTHRU) 
    {
        memptr = (uint8_t*)&(gyroRxFrame->gyroX_H);
    } else if (boardCommState.commMode == GTBCM_GYRO_ACC_PASSTHRU) 
    {
        memptr = (uint8_t*)&(gyroRxFrame->accAddress);
    }

    if (boardCommState.commMode >= GTBCM_GYRO_ACC_FILTER_F){
        gyro_int_to_float(gyroRxFrame);
        filter_data(&rawRateData, &rawAccData, gyroTempData, &filteredData); //profile: this takes 2.45us to run with O3 optimization, before adding biquad at least

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
            case 25:
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
            case 10:
                increment_fft_state();
                break;
            case 33:
                //reset acc tracker
                accTracker = 1; //fallthru for 33, not done on 17
            case 17:
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

    if (boardCommState.commMode != GTBCM_SETUP)
    {
        static int everyOther = 1;
        static int oopsCounter = 0;
        #define RESYNC_COUNTER 100

        //everyother is 16KHz
        if (everyOther-- <= 0)
        {
            append_crc_to_data( (uint32_t *)memptr, (boardCommState.commMode >> 2)-1);
            everyOther = 1; //reset khz counter

            //check if spi is done if not, return
            //if it's not done for RESYNC_COUNTER counts in a row we reset the sync
            if(!spiDoneFlag)
            {
                if( oopsCounter++ < RESYNC_COUNTER )
                {  
                    //give time for spi transfer to happen
                    return;
                }
                else
                {
                    //reset spi and dma for spi since we've not had a reply in a while now
                    cleanup_spi(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, BOARD_COMM_SPI_RST_MSK); //reset sync
                }
            }
            oopsCounter = 0; //reset sync count

            //send the filtered data to the device
            spiDoneFlag = 0; //flag for use during runtime to limit ISR overhead, might be able to remove this completely 
            spi_fire_dma(BOARD_COMM_SPI, BOARD_COMM_TX_DMA, BOARD_COMM_RX_DMA, &boardCommDmaInitStruct, (uint32_t *)&(boardCommState.bufferSize), memptr, bcRxPtr);
            gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); //a quick spike for EXTI
            gpio_write_pin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0); //a quick spike for EXTI
        }
    }
}

void gyro_init(void) 
{
    init_orientation();
    gyroTempData = 0;
    calibratingGyro = 0;    
    gyroSum.x = 0.0f;
    gyroSum.y = 0.0f;
    gyroSum.z = 0.0f;
    gyro_device_init(&gyro_read_done);
}