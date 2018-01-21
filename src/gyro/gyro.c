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
static uint32_t gyro_write_reg(uint8_t reg, uint8_t data);
static uint32_t gyro_verify_write_reg(uint8_t reg, uint8_t data);
static uint32_t gyro_read_data(uint8_t reg, uint8_t *data, uint8_t length);
static uint32_t gyro_slow_read_data(uint8_t reg, uint8_t *data, uint8_t length);
static uint32_t gyro_dma_read_write_data(uint8_t *txData, uint8_t *rxData, uint8_t length);

static void gyro_configure(void)
{
    deviceWhoAmI = 0;
    gyroConfig = mpu6500GyroConfig[0];

    // reset gyro
	gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
	HAL_Delay(80);

    // set gyro clock to Z axis gyro
    gyro_verify_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_CLK_Z);

    // clear low power states
    gyro_write_reg(INVENS_RM_PWR_MGMT_2, 0);

    // disable I2C Interface, clear fifo, and reset sensor signal paths
    // TODO: shouldn't disable i2c on non-spi
    gyro_write_reg(INVENS_RM_USER_CTRL, INVENS_CONST_I2C_IF_DIS | INVENS_CONST_FIFO_RESET | INVENS_CONST_SIG_COND_RESET);

    // set gyro sample divider rate
    gyro_verify_write_reg(INVENS_RM_SMPLRT_DIV, gyroConfig.rateDiv - 1);

    // gyro DLPF config
    gyro_verify_write_reg(INVENS_RM_CONFIG, gyroConfig.gyroDlpf);

    // set gyro full scale to +/- 2000 deg / sec
    gyro_verify_write_reg(INVENS_RM_GYRO_CONFIG, INVENS_CONST_GYRO_FSR_2000DPS << 3 | gyroConfig.gyroDlpfBypass);

    // set accel full scale to +/- 16g
    gyro_verify_write_reg(INVENS_RM_ACCEL_CONFIG, INVENS_CONST_ACC_FSR_16G << 3);

    if (deviceWhoAmI != MPU6000_WHO_AM_I) //6000 is only gyro not to have this function
    {
    	// set the accelerometer dlpf
    	gyro_verify_write_reg(INVENS_RM_ACCEL_CONFIG2, gyroConfig.accDlpfBypass << 3 | gyroConfig.accDlpf);
    	//this function varies between 6000 and 6500+ family
    	// set interrupt pin PP, 50uS pulse, status cleared on INT_STATUS read
    	gyro_verify_write_reg(INVENS_RM_INT_PIN_CFG, INVENS_CONST_INT_RD_CLEAR | INVENS_CONST_BYPASS_EN);
    }
    else
    {
        // set interrupt pin PP, 50uS pulse, status cleared on INT_STATUS read
    	gyro_verify_write_reg(INVENS_RM_INT_PIN_CFG, INVENS_CONST_INT_RD_CLEAR);
    }

    // enable data ready interrupt
    gyro_verify_write_reg(INVENS_RM_INT_ENABLE, INVENS_CONST_DATA_RDY_EN);

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

    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    }
    HAL_Delay(2);

    // TODO: what should these timeouts be?
    HAL_SPI_Transmit(&gyroSPIHandle, &reg, 1, 25);
    HAL_SPI_Transmit(&gyroSPIHandle, &data, 1, 25);

    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
    HAL_Delay(2);

    return 1;
}

static uint32_t gyro_verify_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t attempt, data_verify;

    for (attempt = 0; attempt < 20; attempt++)
    {
    	gyro_write_reg(reg, data);
        HAL_Delay(2);

        gyro_slow_read_data(reg, &data_verify, 1);
        if (data_verify == data)
        {
            return 1;
        }
    }

    error_handler(GYRO_SETUP_COMMUNICATION_FAILIURE);

    return 0;  // this is never reached
}

static uint32_t gyro_read_data(uint8_t reg, uint8_t *data, uint8_t length)
{
    reg |= 0x80;

    // poll until SPI is ready in case of ongoing DMA
    while (HAL_SPI_GetState(&gyroSPIHandle) != HAL_SPI_STATE_READY);

    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    }

    HAL_SPI_Transmit(&gyroSPIHandle, &reg, 1, 100);
    HAL_SPI_Receive(&gyroSPIHandle, data, length, 100);

    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }

    return 1;
}

static uint32_t gyro_slow_read_data(uint8_t reg, uint8_t *data, uint8_t length)
{
    reg |= 0x80;

    // poll until SPI is ready in case of ongoing DMA
    while (HAL_SPI_GetState(&gyroSPIHandle) != HAL_SPI_STATE_READY);

    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    }
    HAL_Delay(1);

    HAL_SPI_Transmit(&gyroSPIHandle, &reg, 1, 100);
    HAL_SPI_Receive(&gyroSPIHandle, data, length, 100);

    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
    HAL_Delay(1);

    return 1;
}

static uint32_t gyro_dma_read_write_data(uint8_t *txData, uint8_t *rxData, uint8_t length)
{
    volatile HAL_DMA_StateTypeDef dmaState = HAL_DMA_GetState(&hdmaGyroSPIRx);
    volatile HAL_SPI_StateTypeDef spiState = HAL_SPI_GetState(&gyroSPIHandle);

    // ensure that both SPI and DMA resources are available, but don't block if they are not
    if (dmaState == HAL_DMA_STATE_READY && spiState == HAL_SPI_STATE_READY)
    {
        if(!GYRO_CS_HARDWARE)
        {
            HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
        }

        HAL_SPI_TransmitReceive_DMA(&gyroSPIHandle, txData, rxData, length);

        return 1;
    }
    else
    {
        return 0;
    }
}