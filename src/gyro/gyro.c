#include "includes.h"
#include "gyro.h"
#include "spi.h"
#include "invensense_register_map.h"
#include "hal_gpio_init.h"

#define GYRO_BUFFER_SIZE 256

//SPI 2 is for the gyro
SPI_HandleTypeDef gyroSPIHandle;
DMA_HandleTypeDef hdmaGyroSPIRx;
DMA_HandleTypeDef hdmaGyroSPITx;
uint8_t gyroSpiRxBuffer[GYRO_BUFFER_SIZE];
uint8_t gyroSpiTxBuffer[GYRO_BUFFER_SIZE];

gyro_device_config_t gyroConfig;
int skipGyro;
float gyroAccData[3];
float gyroTempData;
float gyroRateData[3];
float gyroRateMultiplier;
float gyroTempMultiplier;
float gyroAccMultiplier;

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
static void gyro_exti_init(void);
static void gyro_device_read(void);
static int gyro_device_detect(void);
static uint32_t gyro_write_reg(uint8_t reg, uint8_t data);
static uint32_t gyro_verify_write_reg(uint8_t reg, uint8_t data);
static uint32_t gyro_read_data(uint8_t reg, uint8_t *data, uint8_t length);
static uint32_t gyro_slow_read_data(uint8_t reg, uint8_t *data, uint8_t length);
static uint32_t gyro_dma_read_write_data(uint8_t *txData, uint8_t *rxData, uint8_t length);

static void gyro_configure(void)
{
    deviceWhoAmI = 0;
    gyroConfig = mpu6500GyroConfig[0];

    HAL_Delay(5);

    if (!gyro_device_detect())
    {
        error_handler(GYRO_DETECT_FAILURE);
    }

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

    if (deviceWhoAmI == ICM20601_WHO_AM_I)
    {
        gyroRateMultiplier = 0.1219512195121951f;
        gyroAccMultiplier  = 0.0009765625f;
    }
    else
    {
        gyroRateMultiplier = 0.060975609756098f;
        gyroAccMultiplier  = 0.00048828125f;
    }
    gyroTempData = 0.0030599755201958f;

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
    spi_init(&gyroSPIHandle, GYRO_SPI, baudratePrescaler, SPI_MODE_MASTER, GYRO_SPI_IRQn, GYRO_SPI_ISR_PRE_PRI, GYRO_SPI_ISR_SUB_PRI);
    spi_dma_init(&gyroSPIHandle, &hdmaGyroSPIRx, &hdmaGyroSPITx, GYRO_RX_DMA, GYRO_TX_DMA);
    if(!GYRO_CS_HARDWARE)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}

static int gyro_device_detect(void)
{
    uint8_t attempt, data;

    // reset gyro
    gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
    HAL_Delay(80);
    gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);

    // poll for the who am i register while device resets
    for (attempt = 0; attempt < 25; attempt++)
    {
        HAL_Delay(80);

        gyro_read_data(INVENS_RM_WHO_AM_I, &data, 1);
        switch (data)
        {
        	case MPU6000_WHO_AM_I:
            case MPU6555_WHO_AM_I:
		    case MPU9250_WHO_AM_I:
            case ICM20689_WHO_AM_I:
            case ICM20608G_WHO_AM_I:
		    case ICM20602_WHO_AM_I:
            case ICM20601_WHO_AM_I:
		    case MPU6500_WHO_AM_I:
            	deviceWhoAmI = data;
                return data;
			break;
		    default:
            break;
        }

    }

    return (0);
}

void gyro_init(void) 
{
    skipGyro = 0;
    bzero(gyroAccData,sizeof(gyroAccData));
    gyroTempData = 0;
    bzero(gyroRateData,sizeof(gyroRateData));

    spiIrqCallbackFunctionArray[GYRO_SPI_NUM] = gyro_spi_irq_callback;
    spiCallbackFunctionArray[GYRO_SPI_NUM] = gyro_rx_complete_callback;

    //setup SPI at low speed
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_32);

    //reset and configure gyro
    gyro_configure();

    //setup SPI again at faster speed
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_2);

    //init gyro external interupt
    gyro_exti_init();

}

static void gyro_exti_init(void)
{
    //setup GPIO for EXTI
    hal_gpio_init_pin(GYRO_EXTI_PORT, GYRO_EXTI_PIN, GPIO_MODE_IT_RISING, GPIO_PULLDOWN, 0);
    //EXTI interrupt init
    HAL_NVIC_SetPriority(GYRO_EXTI_IRQn, GYRO_EXTI_ISR_PRE_PRI, GYRO_EXTI_ISR_SUB_PRI);
    HAL_NVIC_EnableIRQ(GYRO_EXTI_IRQn);
}

void gyro_passthrough_start(void) 
{
}

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
    (void)(hspi);
    static int gyroLoopCounter = 0;

    if (gyroLoopCounter-- <= 0)
    {
        //if accDenom is 8, then we should do a switch case for quatenrion math.
        gyroLoopCounter = gyroConfig.accDenom;
        gyroAccData[0] = ((int16_t)((gyroRxFrame.accelX_H << 8) | gyroRxFrame.accelX_L)) * gyroAccMultiplier;
		gyroAccData[1] = ((int16_t)((gyroRxFrame.accelY_H << 8) | gyroRxFrame.accelY_L)) * gyroAccMultiplier;
		gyroAccData[2] = ((int16_t)((gyroRxFrame.accelZ_H << 8) | gyroRxFrame.accelZ_L)) * gyroAccMultiplier;
        gyroTempData   = ((int16_t)((gyroRxFrame.temp_H << 8)   | gyroRxFrame.temp_L))   * gyroTempData + 25;
        //= (TEMP_OUT[15:0]/Temp_Sensitivity) +
        //RoomTemp_Offset
        //where Temp_Sensitivity = 326.8 LSB/ºC and
        //RoomTemp_Offset = 25ºC
        //gyroTempData is gyro temp in C
    }

    gyroRateData[0] = ((int16_t)((gyroRxFrame.gyroX_H << 8) | gyroRxFrame.gyroX_L)) * gyroRateMultiplier;
	gyroRateData[1] = ((int16_t)((gyroRxFrame.gyroY_H << 8) | gyroRxFrame.gyroY_L)) * gyroRateMultiplier;
	gyroRateData[2] = ((int16_t)((gyroRxFrame.gyroZ_H << 8) | gyroRxFrame.gyroZ_L)) * gyroRateMultiplier;

    //gyro read is completes

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