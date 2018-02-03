#include "includes.h"
#include "gyro.h"
#include "spi.h"
#include "invensense_register_map.h"
#include "hal_gpio_init.h"
#include "fast_kalman.h"
#include "imu.h"
#include "board_comm.h"
#include "boothandler.h"
#include "quaternions.h"


volatile uint32_t debug1=0;
volatile uint32_t debug2=0;

volatile int calibratingGyro;
volatile axisData_t gyroSum;
volatile axisData_t gyroCalibrationTrim;

//SPI 2 is for the gyro
SPI_HandleTypeDef gyroSPIHandle;
DMA_HandleTypeDef hdmaGyroSPIRx;
DMA_HandleTypeDef hdmaGyroSPITx;

int skipGyro;

float gyroTempData;
filteredData_t filteredData;

axisData_t rawAccData;
axisData_t rawRateData;
float gyroRateMultiplier = GYRO_DPS_SCALE_2000;
float gyroAccMultiplier = ACC_DPS_SCALE_2000;


static gyroFrame_t gyroRxFrame;
static gyroFrame_t gyroTxFrame;
int32_t deviceWhoAmI = 0;




static void gyro_spi_setup(uint32_t baudratePrescaler)
{
    spi_init(&gyroSPIHandle, GYRO_SPI, baudratePrescaler, SPI_MODE_MASTER, GYRO_SPI_IRQn, GYRO_SPI_ISR_PRE_PRI, GYRO_SPI_ISR_SUB_PRI);
    spi_dma_init(&gyroSPIHandle, &hdmaGyroSPIRx, &hdmaGyroSPITx, GYRO_RX_DMA, GYRO_TX_DMA, GYRO_SPI_RX_DMA_IRQn, GYRO_SPI_TX_DMA_IRQn);

    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}


void gyro_init(void) 
{
    skipGyro = 0;
    gyroTempData = 0;

    gyroSum.x = 0.0f;
    gyroSum.y = 0.0f;
    gyroSum.z = 0.0f;

    calibratingGyro = 0;

    spiCallbackFunctionArray[GYRO_SPI_NUM] = gyro_rx_complete_callback;

    //setup SPI at low speed
    //ICM20601/2 doesn't need slower SPI for this part.
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_8);

    //reset and configure gyro
    gyro_configure();

    //setup SPI again at faster speed
    //SPI_BAUDRATEPRESCALER_2  = 24
    //SPI_BAUDRATEPRESCALER_4  = 16
    //SPI_BAUDRATEPRESCALER_8  = 8
    //SPI_BAUDRATEPRESCALER_16 = 4
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_4);

    //init gyro external interupt
    gyro_exti_init();

}

static void gyro_exti_init(void)
{
    //setup GPIO for EXTI
    hal_gpio_init_pin(GYRO_EXTI_PORT, GYRO_EXTI_PIN, GPIO_MODE_IT_RISING, GPIO_PULLUP, 0);
    //EXTI interrupt init
    HAL_NVIC_SetPriority(GYRO_EXTI_IRQn, GYRO_EXTI_ISR_PRE_PRI, GYRO_EXTI_ISR_SUB_PRI);
    HAL_NVIC_EnableIRQ(GYRO_EXTI_IRQn);
}

// void gyro_passthrough_start(void) 
// {
// }

inline void gyro_exti_callback(void)
{
    //gyro has new data
    HAL_GPIO_EXTI_IRQHandler(GYRO_EXTI_PIN);

    //skip gyro read?
    if (!skipGyro)
    {

        //read all data all the time, but only update ACC data every X times, handled elsewhere now
        gyro_device_read();

    	//update ACC after the rest of the flight code upon the proper denom
    	//if (gyroLoopCounter--==0) {
    	//	gyroLoopCounter = gyroConfig.accDenom;
        //	accgyroDeviceReadAccGyro();
        //} else {
        //	accgyroDeviceReadGyro();
        //}
    }
}




void gyro_rx_complete_callback(SPI_HandleTypeDef *hspi)
{
    uint32_t accTracker = 8; //start at 7, so 8 is run first
    volatile quaternion_buffer_t *quatBuffer = &(quatBufferA); //start working on this buffer
    (void)(hspi); //we don't care about which handle this is as we only have one gyro

    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
    //default rateData
    uint8_t* memptr = (uint8_t*)&filteredData.rateData;

    if (boardCommState.commMode == GTBCM_GYRO_ONLY_PASSTHRU) 
    {
        memptr = (uint8_t*)&gyroTxFrame.gyroX_H;
    } else if (boardCommState.commMode == GTBCM_GYRO_ACC_PASSTHRU) 
    {
        memptr = (uint8_t*)&gyroTxFrame.accAddress;
    }

    if (boardCommState.commMode == GTBCM_GYRO_ACC_FILTER_F || boardCommState.commMode == GTBCM_GYRO_ONLY_FILTER_F || boardCommState.commMode == GTBCM_GYRO_ACC_QUAT_FILTER_F){
        gyro_int_to_float();
        filter_data(&rawRateData,&rawAccData,gyroTempData,&filteredData); //profile: this takes 2.45us to run with O3 optimization, before adding biquad at least
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
    

    static int everyOther = 3;
    if (boardCommState.commMode != GTBCM_SETUP)
    {
        if (everyOther-- < 1)
        {
            #ifdef DEBUG
            everyOther = 3;
            #else
            if (boardCommState.commMode == GTBCM_GYRO_ACC_QUAT_FILTER_F)
            {
                everyOther = 3;
            }
            else
            {
                everyOther = 1;
            }
            #endif

            timeBoardCommSetupIsr = HAL_GetTick();
            //profile:
            //HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
            //HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);

            rewind_board_comm_spi(); //profile: this takes 1.35us to run with O3 optimization
            if (HAL_SPI_GetState(&boardCommSPIHandle) == HAL_SPI_STATE_READY)
            {
                if ( imufCommandRx.command == 0x7f7f7f7F)
                {
                    BootToAddress(THIS_ADDRESS);
                }
                if ( imufCommandRx.command == 0x63636363)
                {
                    calibratingGyro = 1;
                }
                imufCommandRx.command = BC_NONE; //no command
                //transmit from the memptr to save CPU cycles, receive into the command rx struct, saves about 2us of time
                HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, memptr, (uint8_t *)&imufCommandRx, boardCommState.commMode); //profile: this takes 2.14us to run with O3 optimization
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); // a quick spike for EXTI
                HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0); // a quick spike for EXTI
            }
        }
    }
}

static void gyro_device_read(void)
{
    // start read from accel, set high bit to read
    gyroTxFrame.accAddress = INVENS_RM_ACCEL_XOUT_H | 0x80;

    // read 15 bytes, this includes ACC, TEMP, GYRO
    gyro_dma_read_write_data(&gyroTxFrame.accAddress, &gyroRxFrame.accAddress, 15);
}

static uint32_t gyro_write_reg(uint8_t reg, uint8_t data)
{
	uint32_t timeout;

    // poll until SPI is ready in case of ongoing DMA
	for (timeout = 0;timeout<10;timeout++)
    {
		if (HAL_SPI_GetState(&gyroSPIHandle) == HAL_SPI_STATE_READY)
			break;
		HAL_Delay(1);
	}

	if (timeout == 10) {
		return 0;
	}

    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    }
    HAL_Delay(2);

    // TODO: what should these timeouts be?
    HAL_SPI_Transmit(&gyroSPIHandle, &reg, 1, 25);
    HAL_SPI_Transmit(&gyroSPIHandle, &data, 1, 25);

    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
    HAL_Delay(2);

    return 1;
}

void hal_spi_gyro_tx_rx(uint8_t reg, uint8_t *data, uint8_t length) 
{
    HAL_SPI_Transmit(&gyroSPIHandle, &reg, 1, 100);
    HAL_SPI_Receive(&gyroSPIHandle, data, length, 100);
    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}

static uint32_t gyro_dma_read_write_data(uint8_t *txData, uint8_t *rxData, uint8_t length)
{
    volatile HAL_DMA_StateTypeDef dmaState = HAL_DMA_GetState(&hdmaGyroSPIRx);
    volatile HAL_SPI_StateTypeDef spiState = HAL_SPI_GetState(&gyroSPIHandle);

    // ensure that both SPI and DMA resources are available, but don't block if they are not
    if (dmaState == HAL_DMA_STATE_READY && spiState == HAL_SPI_STATE_READY)
    {
        if(GYRO_CS_TYPE  == NSS_SOFT)
        {
            HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
        }

        HAL_SPI_TransmitReceive_DMA(&gyroSPIHandle, txData, rxData, length);

        debug1++;
        return 1;
    }
    else
    {
        debug2++;
        return 0;
    }
}