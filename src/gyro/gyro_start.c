#include "includes.h"
#include "gyro_init.h"

DMA_HandleTypeDef gyroRx;
DMA_HandleTypeDef gyroTx;

void gyro_passthrough_start() 
{
    gyroTx->Instance GYRO
    
    HAL_DMA_Init(&gyroTx)
    HAL_DMA_Init(&gyroRx)

    
    while(1) 
    {
        GPIOB->ODR = 0xFFFF;
        //InlineDigitalHi(GPIOB, GPIO_PIN_5);
        GPIOB->ODR = 0x0000;
        //InlineDigitalLo(GPIOB, GPIO_PIN_5);
    }
}


void imuRxCallback(void)
{
	gyroData[0] = (int32_t)(int16_t)((gyroRxFrame.gyroX_H << 8) | gyroRxFrame.gyroX_L);
	gyroData[1] = (int32_t)(int16_t)((gyroRxFrame.gyroY_H << 8) | gyroRxFrame.gyroY_L);
	gyroData[2] = (int32_t)(int16_t)((gyroRxFrame.gyroZ_H << 8) | gyroRxFrame.gyroZ_L);

    if (deviceWhoAmI == ICM20601_WHO_AM_I)
        InlineUpdateGyro( gyroData, 0.1219512195121951f ); // 1/8.2 is 0.1219512195121951
    else
        InlineUpdateGyro( gyroData, 0.060975609756098f ); // 1/16.4 is 0.060975609756098

        //32,767
    if (accelUpdate)
    {
		accelData[0] = (int32_t)(int16_t)((gyroRxFrame.accelX_H << 8) | gyroRxFrame.accelX_L);
		accelData[1] = (int32_t)(int16_t)((gyroRxFrame.accelY_H << 8) | gyroRxFrame.accelY_L);
		accelData[2] = (int32_t)(int16_t)((gyroRxFrame.accelZ_H << 8) | gyroRxFrame.accelZ_L);

        if (deviceWhoAmI == ICM20601_WHO_AM_I)
            InlineUpdateAcc( accelData, 0.0009765625f); //  1/1024 is 0.0009765625f
        else
            InlineUpdateAcc( accelData, 0.00048828125f); //  1/2048 is 0.00048828125f

	}
}