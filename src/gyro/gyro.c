#include "includes.h"
#include "gyro.h"
#include "spi.h"
#include "invensense_register_map.h"
#include "hal_gpio_init.h"
#include "fast_kalman.h"
#include "quaternions.h"
#include "board_comm.h"


volatile uint32_t debug1=0;
volatile uint32_t debug2=0;

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

//multiple configs can go here, just need one right now
static const gyro_device_config_t gyroConfig = {1, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8};

static gyroFrame_t gyroRxFrame;
static gyroFrame_t gyroTxFrame;
int32_t deviceWhoAmI = 0;

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
static void gyro_int_to_float(void);

static void gyro_configure(void)
{
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
        gyroRateMultiplier = GYRO_DPS_SCALE_4000;
        gyroAccMultiplier  = ACC_DPS_SCALE_4000;
    }
    // set the accelerometer dlpf
    gyro_verify_write_reg(INVENS_RM_ACCEL_CONFIG2, gyroConfig.accDlpfBypass << 3 | gyroConfig.accDlpf);
    //this function varies between 6000 and 6500+ family
    // set interrupt pin PP, 50uS pulse, status cleared on INT_STATUS read
    gyro_verify_write_reg(INVENS_RM_INT_PIN_CFG, INVENS_CONST_INT_RD_CLEAR | INVENS_CONST_BYPASS_EN);
    // enable data ready interrupt
    gyro_verify_write_reg(INVENS_RM_INT_ENABLE, INVENS_CONST_DATA_RDY_EN);

}

static void gyro_spi_setup(uint32_t baudratePrescaler)
{
    spi_init(&gyroSPIHandle, GYRO_SPI, baudratePrescaler, SPI_MODE_MASTER, GYRO_SPI_IRQn, GYRO_SPI_ISR_PRE_PRI, GYRO_SPI_ISR_SUB_PRI);
    spi_dma_init(&gyroSPIHandle, &hdmaGyroSPIRx, &hdmaGyroSPITx, GYRO_RX_DMA, GYRO_TX_DMA, GYRO_SPI_RX_DMA_IRQn, GYRO_SPI_TX_DMA_IRQn);

    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    }
}

static int gyro_device_detect(void)
{
    uint8_t attempt, data;

    data = 0;
    // reset gyro
    gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
    HAL_Delay(80);
    gyro_write_reg(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);

    // poll for the who am i register while device resets
    for (attempt = 0; attempt < 250; attempt++)
    {
        HAL_Delay(80);

        gyro_read_data(INVENS_RM_WHO_AM_I, &data, 1);
        switch (data)
        {
		    case ICM20602_WHO_AM_I:
            case ICM20601_WHO_AM_I:
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
    gyroTempData = 0;

    spiIrqCallbackFunctionArray[GYRO_SPI_NUM] = gyro_spi_irq_callback;
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
    gyro_spi_setup(SPI_BAUDRATEPRESCALER_8);

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
        //only let the F4 have a small window to talk
        if (boardCommState.commMode != GTBCM_SETUP){
            HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);
        }

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

static void gyro_int_to_float(void)
{

    static int gyroLoopCounter = 0;

    if (gyroLoopCounter-- <= 0)
    {
        //if accDenom is 8, then we should do a switch case for quatenrion math.
        gyroLoopCounter = gyroConfig.accDenom;
        rawAccData.x = ((int16_t)((gyroRxFrame.accelX_H << 8) | gyroRxFrame.accelX_L)) * gyroAccMultiplier;
		rawAccData.y = ((int16_t)((gyroRxFrame.accelY_H << 8) | gyroRxFrame.accelY_L)) * gyroAccMultiplier;
		rawAccData.z = ((int16_t)((gyroRxFrame.accelZ_H << 8) | gyroRxFrame.accelZ_L)) * gyroAccMultiplier;
        gyroTempData   = ((int16_t)((gyroRxFrame.temp_H << 8)   | gyroRxFrame.temp_L))   * GYRO_TEMP_MULTIPLIER + 25;
        //= (TEMP_OUT[15:0]/Temp_Sensitivity) +
        //RoomTemp_Offset
        //where Temp_Sensitivity = 326.8 LSB/ºC and
        //RoomTemp_Offset = 25ºC
        //gyroTempMultiplier is gyro temp in C
    }

    rawRateData.x = ((int16_t)((gyroRxFrame.gyroX_H << 8) | gyroRxFrame.gyroX_L)) * gyroRateMultiplier;
	rawRateData.y = ((int16_t)((gyroRxFrame.gyroY_H << 8) | gyroRxFrame.gyroY_L)) * gyroRateMultiplier;
	rawRateData.z = ((int16_t)((gyroRxFrame.gyroZ_H << 8) | gyroRxFrame.gyroZ_L)) * gyroRateMultiplier;

}

void rewind_spi(void)
{
    #define SPI_I2S_DMAReq_Tx               ((uint16_t)0x0002)
    #define SPI_I2S_DMAReq_Rx               ((uint16_t)0x0001)
    #define DMA_CCR1_EN                     ((uint16_t)0x0001)

    // Clear DMA1 global flags
    //SET_BIT(DMA1->IFCR, BOARD_COMM_RX_DMA_FLAG);
    //SET_BIT(DMA1->IFCR, BOARD_COMM_TX_DMA_FLAG);

    // Disable the DMA channels
    //BOARD_COMM_RX_DMA->CCR &= (uint16_t)(~DMA_CCR1_EN);
    //BOARD_COMM_TX_DMA->CCR &= (uint16_t)(~DMA_CCR1_EN);

    // Bring back SPI2 DMAs to start of Rx & Tx buffers -
    // CPAR/CMAR stay the same after disable, no need to
    // `restore` those.
   // BOARD_COMM_RX_DMA->CNDTR = COM_BUFFER_SIZE;
   // BOARD_COMM_TX_DMA->CNDTR = COM_BUFFER_SIZE * 2;

    /* Reset SPI2 (clears TXFIFO). */
    RCC->APB1RSTR |= BOARD_COMM_SPI_RST_MSK;
    RCC->APB1RSTR &= ~BOARD_COMM_SPI_RST_MSK;

    /* Reconfigure SPI2. */
    spi_init(&boardCommSPIHandle, BOARD_COMM_SPI, SPI_BAUDRATEPRESCALER_2, SPI_MODE_SLAVE, BOARD_COMM_SPI_IRQn, BOARD_COMM_SPI_ISR_PRE_PRI, BOARD_COMM_SPI_ISR_SUB_PRI);
    //spi_dma_init(&boardCommSPIHandle, &hdmaBoardCommSPIRx, &hdmaBoardCommSPITx, BOARD_COMM_RX_DMA, BOARD_COMM_TX_DMA, BOARD_COMM_SPI_RX_DMA_IRQn, BOARD_COMM_SPI_TX_DMA_IRQn);
    //HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, COM_BUFFER_SIZE);

    /* Re-enable SPI2 and DMA channels. */
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_RX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_TX_DMA->CCR |= DMA_CCR1_EN;
    //BOARD_COMM_SPI->CR2 |= SPI_I2S_DMAReq_Rx;
    //BOARD_COMM_SPI->CR1 |= SPI_CR1_SPE;
}
void gyro_rx_complete_callback(SPI_HandleTypeDef *hspi)
{
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

    if (boardCommState.commMode == GTBCM_GYRO_ACC_FILTER_F || boardCommState.commMode == GTBCM_GYRO_ONLY_FILTER_F){
        gyro_int_to_float();
        filter_data(&rawRateData,&rawAccData,gyroTempData,&filteredData); //profile: this takes 2.45us to run with O3 optimization
    }

    if (boardCommState.commMode == GTBCM_GYRO_ACC_QUAT_FILTER_F){
        generate_quaterions(&rawRateData,&rawAccData,&filteredData);
    }

    static int everyOther = 1;
    if (everyOther-- < 1)
    {
        everyOther = 1;
        if (boardCommState.commMode != GTBCM_SETUP){

            //this line is no good, we need to add the flags to the data structure
            memcpy(boardCommSpiTxBuffer+1, memptr, boardCommState.commMode); //profile: this takes 3.84us to run with O3 optimization
            boardCommSpiTxBuffer[0] = 'h';

            //profile:
            //HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);
            //HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0);

            rewind_spi(); //profile: this takes 1.35us to run with O3 optimization
            HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, boardCommSpiTxBuffer, boardCommSpiRxBuffer, boardCommState.commMode); //profile: this takes 2.14us to run with O3 optimization
            HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1);

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

void writeGyroPin(void) 
{
    // poll until SPI is ready in case of ongoing DMA
    while (HAL_SPI_GetState(&gyroSPIHandle) != HAL_SPI_STATE_READY);

    if(GYRO_CS_TYPE  == NSS_SOFT)
    {
        HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    }
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

static uint32_t gyro_read_data(uint8_t reg, uint8_t *data, uint8_t length)
{
    reg |= 0x80;
    writeGyroPin(); 
    hal_spi_gyro_tx_rx(reg, data, length);
    return 1;
}

static uint32_t gyro_slow_read_data(uint8_t reg, uint8_t *data, uint8_t length)
{
    reg |= 0x80;
    writeGyroPin(); 
    HAL_Delay(1);
    hal_spi_gyro_tx_rx(reg, data, length);
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