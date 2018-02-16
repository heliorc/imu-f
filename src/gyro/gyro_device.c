#include "includes.h"
#include "invensense_register_map.h"
#include "gyro_device.h"
#include "fft.h"

//multiple configs can go here, just need one right now
const gyro_device_config_t gyroConfig = {1, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8};

gyroFrame_t gyroRxFrame;
gyroFrame_t gyroTxFrame;

uint8_t *gyroRxFramePtr;
uint8_t *gyroTxFramePtr;

volatile int gyroReadDone = 0;
#define GYRO_READ_TIMEOUT 20

float gyroRateMultiplier = GYRO_DPS_SCALE_2000;
float gyroAccMultiplier = ACC_DPS_SCALE_2000;

volatile gyro_read_done_t gyro_read_done_callback;

SPI_InitTypeDef gyroSpiInitStruct;
DMA_InitTypeDef gyroDmaInitStruct;

extern void arm_bitreversal_32(uint32_t * pSrc, const uint16_t bitRevLen, const uint16_t * pBitRevTable);

static void gyro_read_reg(uint8_t reg, uint8_t data);
static void gyro_write_reg(uint8_t reg, uint8_t data);

static int gyro_read_reg_setup(uint8_t reg, uint8_t data, uint8_t* returnedData);
static int gyro_write_reg_setup(uint8_t reg, uint8_t data, uint8_t* returnedData);
static void gyro_spi_transmit_receive(uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t size);
static void gyro_cleanup_spi(void);
static void gyro_cs_lo(void);
static void gyro_cs_hi(void);
static void gyro_spi_init(void);
static void gyro_setup_exti_fn(gyroFrame_t* gyroRxFrame);

static void super_error(uint32_t time)
{
    volatile int cat = 1;
    single_gpio_init(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN_SRC, BOOTLOADER_CHECK_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 0);
    while(1)
    {
        cat++;
        gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);
        delay_ms(time);
        gpio_write_pin(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 0);
        delay_ms(time);
    }
}

inline static void gyro_cs_lo(void)
{
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 0);
}

inline static void gyro_cs_hi(void)
{
    gpio_write_pin(GYRO_CS_PORT, GYRO_CS_PIN, 1);
}

static void gyro_cleanup_spi(void)
{
    //clear DMA flags
    DMA_ClearFlag(GYRO_ALL_DMA_FLAGS);

    //disable DMAs
    DMA_Cmd(GYRO_TX_DMA,DISABLE);
    DMA_Cmd(GYRO_RX_DMA,DISABLE);  

    //disable SPI DMA requests
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Tx, DISABLE);
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

    //disable SPI
    SPI_Cmd(GYRO_SPI, DISABLE);
}

void GYRO_SPI_RX_DMA_HANDLER(void)
{
    if(DMA_GetITStatus(GYRO_RX_DMA_FLAG_TC))
    {
        gyro_cs_hi();
        gyro_cleanup_spi();
        gyro_read_done_callback(&gyroRxFrame);
        DMA_ClearITPendingBit(GYRO_RX_DMA_FLAG_TC);         
    }
}

static void gyro_spi_init(void)
{
    GPIO_InitTypeDef gpioInitStruct;
    SPI_InitTypeDef spiInitStruct;
    DMA_InitTypeDef dmaInitStruct;
    NVIC_InitTypeDef nvicInitStruct;

    //config pins
    gpioInitStruct.GPIO_Pin = GYRO_CS_PIN;
    gpioInitStruct.GPIO_Mode = GPIO_Mode_OUT;
    gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInitStruct.GPIO_OType = GPIO_OType_PP;
    gpioInitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GYRO_CS_PORT, &gpioInitStruct);

    //set default CS state (high)
    GPIO_SetBits(GYRO_CS_PORT, GYRO_CS_PIN);

    gpioInitStruct.GPIO_Mode = GPIO_Mode_AF; 
    gpioInitStruct.GPIO_Pin = GYRO_SCK_PIN;
    GPIO_Init(GYRO_SCK_PORT, &gpioInitStruct);

    gpioInitStruct.GPIO_Pin = GYRO_MISO_PIN;
    GPIO_Init(GYRO_MISO_PORT, &gpioInitStruct);

    gpioInitStruct.GPIO_Pin = GYRO_MOSI_PIN;
    GPIO_Init(GYRO_MOSI_PORT, &gpioInitStruct);

    //set AF map
    GPIO_PinAFConfig(GYRO_SCK_PORT, GYRO_SCK_PIN_SRC, GYRO_SCK_ALTERNATE);
    GPIO_PinAFConfig(GYRO_MISO_PORT, GYRO_MISO_PIN_SRC, GYRO_MISO_ALTERNATE);
    GPIO_PinAFConfig(GYRO_MOSI_PORT, GYRO_MOSI_PIN_SRC, GYRO_MOSI_ALTERNATE);

    //config SPI
    spiInitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInitStruct.SPI_Mode = SPI_Mode_Master;
    spiInitStruct.SPI_DataSize = SPI_DataSize_8b;
    spiInitStruct.SPI_CPOL = SPI_CPOL_High;
    spiInitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    spiInitStruct.SPI_NSS = SPI_NSS_Soft;
    spiInitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    spiInitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    spiInitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(GYRO_SPI, &spiInitStruct);
    SPI_RxFIFOThresholdConfig(GYRO_SPI,SPI_RxFIFOThreshold_QF);

    //set DMA to default state
    DMA_DeInit(GYRO_TX_DMA);
    DMA_DeInit(GYRO_RX_DMA);

    dmaInitStruct.DMA_M2M = DMA_M2M_Disable;
    dmaInitStruct.DMA_Mode = DMA_Mode_Normal;
    dmaInitStruct.DMA_Priority = DMA_Priority_Medium;
    dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralDST;

    dmaInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t)&GYRO_SPI->DR;

    dmaInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStruct.DMA_MemoryBaseAddr = 0; //this is set later when we fire the DMA

    dmaInitStruct.DMA_BufferSize = 1;     //this is set later when we fire the DMA

    DMA_Init(GYRO_TX_DMA, &dmaInitStruct);

    dmaInitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;

    DMA_Init(GYRO_RX_DMA, &dmaInitStruct);

    //setup interrupt
    nvicInitStruct.NVIC_IRQChannel = GYRO_SPI_RX_DMA_IRQn;
    nvicInitStruct.NVIC_IRQChannelPreemptionPriority = GYRO_SPI_DMA_RX_PRE_PRI;
    nvicInitStruct.NVIC_IRQChannelSubPriority = GYRO_SPI_DMA_RX_SUB_PRI;
    nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInitStruct);
    DMA_ITConfig(GYRO_RX_DMA, DMA_IT_TC, ENABLE);
}

static void gyro_spi_transmit_receive(uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t size)
{     

    //set buffer size
    DMA_SetCurrDataCounter(GYRO_TX_DMA, size);
    DMA_SetCurrDataCounter(GYRO_RX_DMA, size);

    //set buffer
    GYRO_TX_DMA->CMAR = (uint32_t)txBuffer;
    GYRO_RX_DMA->CMAR = (uint32_t)rxBuffer;

    //enable DMA SPI streams
    DMA_Cmd(GYRO_TX_DMA, ENABLE);
    DMA_Cmd(GYRO_RX_DMA, ENABLE);

    //enable  CS
    gyro_cs_lo();

    //enable DMA SPI requests
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_DMACmd(GYRO_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

    //enable and send
    SPI_Cmd(GYRO_SPI, ENABLE);

}

void GYRO_EXTI_HANDLER(void)
{
    //make sure that interrupt flag is set
    if (EXTI_GetITStatus(GYRO_EXTI_LINE) != RESET)
    {
        //what reg do we want to read from? for ICM gyros, add 0x80 to read instead of write
        gyroTxFrame.accAddress = INVENS_RM_ACCEL_XOUT_H | 0x80;
        //set cs pin
        gyro_cs_lo();
        //start the dma transfer
        gyro_spi_transmit_receive(gyroTxFramePtr, gyroRxFramePtr, 15);
        //clear interrupt bit
        EXTI_ClearITPendingBit(GYRO_EXTI_LINE);
    }
}

static int gyro_read_reg_setup(uint8_t reg, uint8_t data, uint8_t* returnedData)
{
    return gyro_write_reg_setup(reg | 0x80, data, returnedData);
}

static int gyro_write_reg_setup(uint8_t reg, uint8_t data, uint8_t* returnedData)
{
    uint32_t timeoutCheck = millis();
    //writing 2 bytes, reg and data, anything that's read back will be returned
    gyroTxFrame.accAddress = reg;
    gyroTxFrame.accelX_H = data;
    //set read done check to 0
    gyroReadDone = 0;
    //drive cs low to enable chip
    gyro_cs_lo();
    //start the dma transfer
    gyro_spi_transmit_receive(gyroTxFramePtr, gyroRxFramePtr, 2);
    while(!gyroReadDone)
    {
        if(millis() - timeoutCheck > GYRO_READ_TIMEOUT)
        {
            //GYRO_READ_TIMEOUT ms max, read failed, cleanup spi and return 0
            gyro_cs_hi();
            gyro_cleanup_spi();
            return 0;
        }
    }

    //success
    if (returnedData)
    {
        *returnedData = gyroRxFrame.accelX_H;
    }
    return 1;

}

static uint32_t gyro_verify_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t attempt, data_verify;

    for (attempt = 0; attempt < 20; attempt++)
    {
    	gyro_write_reg_setup(reg, data, NULL);
        delay_us(100);
        gyro_read_reg_setup(reg, data, &data_verify);
        if (data_verify == data)
        {
            return 1;
        }
    }

    return 0;
}

static int gyro_device_detect(void)
{
    uint32_t attempt;
    uint8_t data = 0;
    // reset gyro
    gyro_write_reg_setup(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET, NULL);
    delay_ms(120);

    // poll for the who am i register while device resets
    for (attempt = 0; attempt < 500; attempt++)
    {
        delay_ms(2);
        gyro_read_reg_setup(INVENS_RM_WHO_AM_I, data, &data);
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
    }

    // set gyro clock to Z axis gyro
    gyro_verify_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_CLK_Z);

    // clear low power states
    gyro_write_reg_setup(INVENS_RM_PWR_MGMT_2, 0, NULL);

    // disable I2C Interface, clear fifo, and reset sensor signal paths
    // TODO: shouldn't disable i2c on non-spi
    gyro_write_reg_setup(INVENS_RM_USER_CTRL, INVENS_CONST_I2C_IF_DIS | INVENS_CONST_FIFO_RESET | INVENS_CONST_SIG_COND_RESET, NULL);

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

static void gyro_setup_exti_fn(gyroFrame_t* gyroRxFrame)
{
    (void)(gyroRxFrame);
    gyroReadDone = 1; //used for blocking reads
}

void gyro_device_init(gyro_read_done_t readFn) 
{
    //set uint8_t ptrs so we don't have to type cast
    gyroRxFramePtr = (uint8_t *)&gyroRxFrame;
    gyroTxFramePtr = (uint8_t *)&gyroTxFrame;

    //no callback for setup
    gyro_read_done_callback = gyro_setup_exti_fn;
    //setup gyro 
    gyro_spi_init();
    //reset and configure gyro
    gyro_configure();

    //setup new read callback
    gyro_read_done_callback = readFn;
    delay_us(100);
    //setup EXTI last
    gpio_exti_init(GYRO_EXTI_PORT, GYRO_EXTI_PORT_SRC, GYRO_EXTI_PIN, GYRO_EXTI_PIN_SRC, GYRO_EXTI_LINE, EXTI_Trigger_Rising, GYRO_EXTI_IRQn, GYRO_EXTI_ISR_PRE_PRI, GYRO_EXTI_ISR_SUB_PRI);
}


