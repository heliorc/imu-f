#include "includes.h"
#include "gyro.h"
#include "spi.h"
#include "invensense_register_map.h"

gyro_device_config_t gyroConfig;

typedef struct gyroFrame
{
    uint8_t accAddress;  // needed to start rx/tx transfer when sending address
    uint8_t accelX_H;
    uint8_t accelX_L;
    uint8_t accelY_H;
    uint8_t accelY_L;
    uint8_t accelZ_H;
    uint8_t accelZ_L;
    uint8_t temp_H;
    uint8_t temp_L;
    uint8_t gyroX_H;
    uint8_t gyroX_L;
    uint8_t gyroY_H;
    uint8_t gyroY_L;
    uint8_t gyroZ_H;
    uint8_t gyroZ_L;
} __attribute__((__packed__)) gyroFrame_t;

//multiple configs can go here, just need one right now
static const gyro_device_config_t mpu6500GyroConfig[] =
{
    [0] = {1, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
};


static gyroFrame_t gyroRxFrame;
static gyroFrame_t gyroTxFrame;
int32_t deviceWhoAmI;

static void gyro_configure(void);
static void gyro_spi_setup(uint32_t baudratePrescaler);

static void gyro_configure(void)
{
    deviceWhoAmI = 0;
    gyroConfig = mpu6500GyroConfig[0];
}

static void gyro_spi_setup(uint32_t baudratePrescaler)
{
    spi_init(&gyroSPIHandle, GYRO_SPI, baudratePrescaler, SPI_MODE_MASTER, GYRO_SPI_IRQn, 1, 2);
    spi_dma_init(&gyroSPIHandle, &hdmaGyroSPIRx, &hdmaGyroSPITx, GYRO_RX_DMA, GYRO_TX_DMA);
    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}

void gyro_init(void) 
{

    spiIrqCallbackFunctionArray[GYRO_SPI_NUM] = gyro_spi_irq_callback;
    spiCallbackFunctionArray[GYRO_SPI_NUM] = gyro_rx_complete_callback;

    //setup SPI at low speed
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_32);

    //reset and configure gyro
    gyro_configure();

    //setup SPI again at faster speed
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_2);

}

void gyro_passthrough_start() 
{
    /*
    while(1) 
    {
        GPIOB->ODR = 0xFFFF;
        //InlineDigitalHi(GPIOB, GPIO_PIN_5);
        GPIOB->ODR = 0x0000;
        //InlineDigitalLo(GPIOB, GPIO_PIN_5);
    }
    */
}

void gyro_rx_complete_callback(SPI_HandleTypeDef *hspi)
{

}

//todo register callback function
void imuRxCallback(void)
{
    /*
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
    */
}