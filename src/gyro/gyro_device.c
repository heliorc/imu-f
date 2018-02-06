#include "includes.h"
#include "invensense_register_map.h"
#include "gyro_device.h"

//multiple configs can go here, just need one right now
const gyro_device_config_t gyroConfig = {1, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8};

gyroFrame_t gyroRxFrame;
gyroFrame_t gyroTxFrame;

uint8_t *gyroRxFramePtr;
uint8_t *gyroTxFramePtr;

float gyroRateMultiplier = GYRO_DPS_SCALE_2000;
float gyroAccMultiplier = ACC_DPS_SCALE_2000;

volatile gyro_read_done_t gyro_read_callback;

SPI_InitTypeDef gyroSpiInitStruct;
DMA_InitTypeDef gyroDmaInitStruct;

static uint8_t gyro_read_reg(uint8_t reg, uint8_t data);
static uint8_t gyro_write_reg(uint8_t reg, uint8_t data);


static void gyro_device_read(void)
{
    // start read from accel, set high bit to read
    gyroTxFrame.accAddress = INVENS_RM_ACCEL_XOUT_H | 0x80;
    // read 15 bytes, this includes ACC, TEMP, GYRO
    gyro_read_callback((uint32_t)&gyroTxFrame.accAddress, &gyroRxFrame.accAddress, 15);
}

static uint8_t gyro_read_reg(uint8_t reg, uint8_t data)
{
    return gyro_write_reg(reg | 0x80, data);
}


int spi_transfer_blocking(SPI_TypeDef* spi, const uint8_t* txBuff, uint8_t* rxBuff, int len)
{
    //TODO: Check timeout
    while (len--)
    {
        while(SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET); //tx buffer empyt?
        SPI_SendData8(spi, *(txBuff++));
        while( spi->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
        *(rxBuff++) = SPI_ReceiveData8(spi);
    }

    return 1;
}


static uint8_t gyro_write_reg(uint8_t reg, uint8_t data)
{
    //writing 2 bytes, reg and data, anything that's read back will be returned
    //set buffer
    uint8_t buff1[20];
    uint8_t buff2[20];
    buff1[0] = reg;
    buff1[1] = data;
    buff2[0] = 0;
    buff2[1] = 0;
    //gyroTxFramePtr[0] = reg;
    //gyroTxFramePtr[1] = data;
    uint32_t size = 2;

    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 0); //high to deactive cs on gyro
    spi_transfer_blocking(GYRO_SPI, buff1, buff2, 2);
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 1); //high to deactive cs on gyro
    return buff2[1];

/*
    uint32_t timeout = millis();
    // return 0;
    //send/receive data, return zero if we time out
     //set cs pin
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 0); //low to active cs on gyro
    //start the dma transfer
    volatile uint32_t mouse1 = DMA_GetFlagStatus(GYRO_TX_DMA_FLAG_TC);
    //spi_fire_dma(GYRO_SPI, GYRO_TX_DMA, GYRO_RX_DMA, &gyroDmaInitStruct, &size, buff1, buff2);
    volatile uint32_t mouse2 = DMA_GetFlagStatus(GYRO_TX_DMA_FLAG_TC);
    while (DMA_GetFlagStatus(GYRO_TX_DMA_FLAG_TC) == RESET)
    {
        if (millis() - timeout > 80) //10 MS TIMEOUT
        {
            volatile uint32_t timeww = millis();
            cleanup_spi(GYRO_SPI, GYRO_TX_DMA, GYRO_RX_DMA, GYRO_TX_DMA_FLAG_GL, GYRO_RX_DMA_FLAG_GL, GYRO_SPI_RST_MSK);
            gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 1); //high to deactive cs on gyro
            return 0;
        }
    }
    volatile uint32_t mouse3 = DMA_GetFlagStatus(GYRO_TX_DMA_FLAG_TC);
    //if we got here than the TX/RX was a success
    cleanup_spi(GYRO_SPI, GYRO_TX_DMA, GYRO_RX_DMA, GYRO_TX_DMA_FLAG_GL, GYRO_RX_DMA_FLAG_GL, GYRO_SPI_RST_MSK);
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 1); //high to deactive cs on gyro
    return buff2[1];
*/
}

static uint32_t gyro_verify_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t attempt, data_verify;

    for (attempt = 0; attempt < 20; attempt++)
    {
    	gyro_write_reg(reg, data);
        delay_ms(2);
        data_verify = gyro_read_reg(reg, data);
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
    uint32_t attempt;
    uint8_t data = 0;
    // reset gyro
    gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
    delay_ms(150);
    // poll for the who am i register while device resets
    for (attempt = 0; attempt < 100; attempt++)
    {
        delay_ms(2);
        data = gyro_read_reg(INVENS_RM_WHO_AM_I, data);
        if (data == ICM20601_WHO_AM_I)
        {
            gyroRateMultiplier = GYRO_DPS_SCALE_4000;
            gyroAccMultiplier  = ACC_DPS_SCALE_4000;
            return(1);
        }
        else if (data == ICM20602_WHO_AM_I)
        {
            gyroRateMultiplier = GYRO_DPS_SCALE_2000;
            gyroAccMultiplier  = ACC_DPS_SCALE_2000;
            return(1);
        }
    }
    return(0);
}

static void gyro_configure(void)
{
    delay_ms(5);
    if (!gyro_device_detect())
    {
        // error_handler(GYRO_DETECT_FAILURE);
        volatile int failed = 1;
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
    // setup GYRO spi mappings and gpio init
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 1);

    single_gpio_init(GYRO_MISO_PORT, GYRO_MISO_PIN_SRC, GYRO_MISO_PIN, GYRO_MISO_ALTERNATE, GPIO_Mode_AF,  GPIO_OType_PP, GPIO_PuPd_DOWN);
    single_gpio_init(GYRO_MOSI_PORT, GYRO_MOSI_PIN_SRC, GYRO_MOSI_PIN, GYRO_MOSI_ALTERNATE, GPIO_Mode_AF,  GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(GYRO_SCK_PORT,  GYRO_SCK_PIN_SRC,  GYRO_SCK_PIN,  GYRO_SCK_ALTERNATE,  GPIO_Mode_AF,  GPIO_OType_PP, GPIO_PuPd_NOPULL);
    single_gpio_init(GYRO_CS_PORT,   GYRO_CS_PIN_SRC,   GYRO_CS_PIN,   GYRO_CS_ALTERNATE,   GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 1);

    //setup NSS GPIO if need be then init SPI and DMA for the SPI based on NSS type
    gpio_exti_init(GYRO_EXTI_PORT, GYRO_EXTI_PORT_SRC, GYRO_EXTI_PIN, GYRO_EXTI_PIN_SRC, GYRO_EXTI_LINE, EXTI_Trigger_Rising, GYRO_EXTI_IRQn, GYRO_EXTI_ISR_PRE_PRI, GYRO_EXTI_ISR_SUB_PRI);
    spi_init(&gyroSpiInitStruct, &gyroDmaInitStruct, GYRO_SPI, SPI_Mode_Master, SPI_NSS_Soft, SPI_CPOL_High, SPI_CPHA_2Edge, SPI_BaudRatePrescaler_32); 
}

void gyro_device_init(gyro_read_done_t readFn) 
{
    //set uint8_t ptrs so we don't have to type cast
    gyroRxFramePtr = (uint8_t *)&gyroRxFrame;
    gyroTxFramePtr = (uint8_t *)&gyroTxFrame;

    gyro_read_callback = readFn;
    spiCallbackFunctionArray[GYRO_SPI_NUM] = gyro_device_read;
    //setup gyro 
    gyro_spi_setup();
    //reset and configure gyro
    gyro_configure();
}


