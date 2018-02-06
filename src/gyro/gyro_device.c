#include "includes.h"
#include "invensense_register_map.h"
#include "gyro_device.h"

//multiple configs can go here, just need one right now
const gyro_device_config_t gyroConfig = {1, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8};

gyroFrame_t gyroRxFrame;
gyroFrame_t gyroTxFrame;

float gyroRateMultiplier = GYRO_DPS_SCALE_2000;
float gyroAccMultiplier = ACC_DPS_SCALE_2000;

volatile gyro_read_done_t gyro_read_callback;

SPI_InitTypeDef gyroSpiInitStruct;
DMA_InitTypeDef gyroDmaInitStruct;


static void gyro_device_read(void)
{
    // start read from accel, set high bit to read
    gyroTxFrame.accAddress = INVENS_RM_ACCEL_XOUT_H | 0x80;
    // read 15 bytes, this includes ACC, TEMP, GYRO
    gyro_read_callback((uint32_t)&gyroTxFrame.accAddress, &gyroRxFrame.accAddress, 15);
}

static uint32_t gyro_write_reg(uint8_t reg, uint8_t data)
{
	uint32_t timeout;

    // poll until SPI is ready in case of ongoing DMA
	for (timeout = 0;timeout<10;timeout++)
    {
		// if (HAL_SPI_GetState(&gyroSPIHandle) == HAL_SPI_STATE_READY)
		// 	break;
		// HAL_Delay(1);
	}

	if (timeout == 10) {
		return 0;
	}

    // if(GYRO_CS_TYPE  == NSS_SOFT)
    // {
    //     HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    // }
    // HAL_Delay(2);

    // TODO: what should these timeouts be?
    // HAL_SPI_Transmit(&gyroSPIHandle, &reg, 1, 25);
    // HAL_SPI_Transmit(&gyroSPIHandle, &data, 1, 25);

    // if(GYRO_CS_TYPE  == NSS_SOFT)
    // {
    //     HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    // }
    // HAL_Delay(2);

    return 1;
}

static uint32_t gyro_verify_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t attempt, data_verify;

    for (attempt = 0; attempt < 20; attempt++)
    {
    	gyro_write_reg(reg, data);
        // HAL_Delay(2);
        // gyro_device_read(reg, &data_verify, 1, 1);
        if (data_verify == data)
        {
            return 1;
        }
    }

    // error_handler(GYRO_SETUP_COMMUNICATION_FAILIURE);

    return 0;  // this is never reached
}

static int gyro_device_detect(void)
{
    uint8_t attempt, data = 0;
    // reset gyro
    gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
    // HAL_Delay(80);
    // poll for the who am i register while device resets
    for (attempt = 0; attempt < 250; attempt++)
    {
        // HAL_Delay(2);
        // gyro_device_read(INVENS_RM_WHO_AM_I, &data, 1, 0);
        if (data == ICM20601_WHO_AM_I) {
            gyroRateMultiplier = GYRO_DPS_SCALE_4000;
            gyroAccMultiplier  = ACC_DPS_SCALE_4000;
            return(1);
        }
    }
    return(0);
}

static void gyro_configure(void)
{
    // HAL_Delay(5);
    if (!gyro_device_detect())
    {
        // error_handler(GYRO_DETECT_FAILURE);
    }

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

    // set the accelerometer dlpf
    gyro_verify_write_reg(INVENS_RM_ACCEL_CONFIG2, gyroConfig.accDlpfBypass << 3 | gyroConfig.accDlpf);
    //this function varies between 6000 and 6500+ family
    // set interrupt pin PP, 50uS pulse, status cleared on INT_STATUS read
    gyro_verify_write_reg(INVENS_RM_INT_PIN_CFG, INVENS_CONST_INT_RD_CLEAR | INVENS_CONST_BYPASS_EN);
    // enable data ready interrupt
    gyro_verify_write_reg(INVENS_RM_INT_ENABLE, INVENS_CONST_DATA_RDY_EN);
}


static void gyro_spi_setup(void)
{
    // setup board_comm spi mappings and gpio init
    single_gpio_init(GYRO_MISO_PORT, GYRO_MISO_PIN, GYRO_MISO_PIN, GYRO_MISO_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(GYRO_MOSI_PORT, GYRO_MOSI_PIN, GYRO_MOSI_PIN, GYRO_MOSI_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(GYRO_SCK_PORT,  GYRO_SCK_PIN,  GYRO_SCK_PIN,  GYRO_SCK_ALTERNATE,  GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    //setup NSS GPIO if need be then init SPI and DMA for the SPI based on NSS type
    gpio_exti_init(GYRO_EXTI_PORT, GYRO_EXTI_PORT_SRC, GYRO_EXTI_PIN, GYRO_EXTI_PIN_SRC, GYRO_EXTI_LINE, EXTI_Trigger_Rising, GYRO_EXTI_IRQn, GYRO_EXTI_ISR_PRE_PRI, GYRO_EXTI_ISR_SUB_PRI);
    spi_init(&gyroSpiInitStruct, &gyroDmaInitStruct, GYRO_SPI, SPI_Mode_Master, SPI_NSS_Soft); 
}

void gyro_device_init(gyro_read_done_t readFn) 
{
    gyro_read_callback = readFn;
    spiCallbackFunctionArray[GYRO_SPI_NUM] = gyro_device_read;
    //setup gyro 
    gyro_spi_setup();
    //reset and configure gyro
    gyro_configure();
}


