#include "includes.h"

//multiple configs can go here, just need one right now
static const gyro_device_config_t gyroConfig = {1, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8};


float gyroRateMultiplier = GYRO_DPS_SCALE_2000;
float gyroAccMultiplier = ACC_DPS_SCALE_2000;

SPI_InitTypeDef gyroSpiInitStruct;

void writeGyroPin(void) 
{
    // poll until SPI is ready in case of ongoing DMA
    while (HAL_SPI_GetState(&gyroSPIHandle) != HAL_SPI_STATE_READY);

    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    }
}

static uint32_t gyro_read_data(uint8_t reg, uint8_t *data, uint8_t length, uint32_t halDelay)
{
    reg |= 0x80;
    writeGyroPin(); 
    if (halDelay) {
        HAL_Delay(halDelay);
    }
    hal_spi_gyro_tx_rx(reg, data, length);
    if (halDelay) {
        HAL_Delay(halDelay);
    }
    return 1;
}


static uint32_t gyro_verify_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t attempt, data_verify;

    for (attempt = 0; attempt < 20; attempt++)
    {
    	gyro_write_reg(reg, data);
        HAL_Delay(2);
        gyro_read_data(reg, &data_verify, 1, 1);
        if (data_verify == data)
        {
            return 1;
        }
    }

    error_handler(GYRO_SETUP_COMMUNICATION_FAILIURE);

    return 0;  // this is never reached
}

static int gyro_device_detect(void)
{
    uint8_t attempt, data = 0;
    // reset gyro
    gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
    HAL_Delay(80);
    // poll for the who am i register while device resets
    for (attempt = 0; attempt < 250; attempt++)
    {
        HAL_Delay(2);
        gyro_read_data(INVENS_RM_WHO_AM_I, &data, 1, 0);
        if (data == ICM20601_WHO_AM_I) {
            gyroRateMultiplier = GYRO_DPS_SCALE_4000;
            gyroAccMultiplier  = ACC_DPS_SCALE_4000;
            return(1);
        }
    }
    return(0);
}

static void gyro_configure(gyro_device_config_t* config)
{
    HAL_Delay(5);
    if (!gyro_device_detect())
    {
        error_handler(GYRO_DETECT_FAILURE);
    }

    // set gyro clock to Z axis gyro
    gyro_verify_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_CLK_Z);

    // clear low power states
    gyro_write_reg(INVENS_RM_PWR_MGMT_2, 0);

    // disable I2C Interface, clear fifo, and reset sensor signal paths
    // TODO: shouldn't disable i2c on non-spi
    gyro_write_reg(INVENS_RM_USER_CTRL, INVENS_CONST_I2C_IF_DIS | INVENS_CONST_FIFO_RESET | INVENS_CONST_SIG_COND_RESET);

    // set gyro sample divider rate
    gyro_verify_write_reg(INVENS_RM_SMPLRT_DIV, config->rateDiv - 1);

    // gyro DLPF config
    gyro_verify_write_reg(INVENS_RM_CONFIG, config->gyroDlpf);

    // set gyro full scale to +/- 2000 deg / sec
    gyro_verify_write_reg(INVENS_RM_GYRO_CONFIG, INVENS_CONST_GYRO_FSR_2000DPS << 3 | config->gyroDlpfBypass);

    // set accel full scale to +/- 16g
    gyro_verify_write_reg(INVENS_RM_ACCEL_CONFIG, INVENS_CONST_ACC_FSR_16G << 3);

    // set the accelerometer dlpf
    gyro_verify_write_reg(INVENS_RM_ACCEL_CONFIG2, config->accDlpfBypass << 3 | config->accDlpf);
    //this function varies between 6000 and 6500+ family
    // set interrupt pin PP, 50uS pulse, status cleared on INT_STATUS read
    gyro_verify_write_reg(INVENS_RM_INT_PIN_CFG, INVENS_CONST_INT_RD_CLEAR | INVENS_CONST_BYPASS_EN);
    // enable data ready interrupt
    gyro_verify_write_reg(INVENS_RM_INT_ENABLE, INVENS_CONST_DATA_RDY_EN);

}

void gyro_device_init(gyro_device_config_t* config) 
{
    gyroTempData = 0;

    gyroSum.x = 0.0f;
    gyroSum.y = 0.0f;
    gyroSum.z = 0.0f;

    calibratingGyro = 0;

    spiIrqCallbackFunctionArray[GYRO_SPI_NUM] = gyro_spi_irq_callback;
    spiCallbackFunctionArray[GYRO_SPI_NUM] = gyro_rx_complete_callback;

    //setup SPI at low speed
    //ICM20601/2 doesn't need slower SPI for this part.
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_8);

    //reset and configure gyro
    gyro_configure(config);

    //setup SPI again at faster speed
    //SPI_BAUDRATEPRESCALER_2  = 24
    //SPI_BAUDRATEPRESCALER_4  = 16
    //SPI_BAUDRATEPRESCALER_8  = 8
    //SPI_BAUDRATEPRESCALER_16 = 4
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_4);

    //init gyro external interupt
    gyro_exti_init();

}


