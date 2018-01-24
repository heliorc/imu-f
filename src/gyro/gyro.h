#pragma once

extern volatile uint32_t debug1;
extern volatile uint32_t debug2;

typedef struct axisData
{
    float x;
    float y;
    float z;
} __attribute__((__packed__)) axisData_t;

typedef struct filteredData
{
    axisData_t rateData;
    axisData_t accData;
    float tempC;
    float quaternion[4];
} __attribute__((__packed__)) filteredData_t;

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

typedef struct gyro_data {
    uint8_t rateDiv;
    uint8_t gyroDlpf;
    uint8_t gyroDlpfBypass;
    uint8_t accDlpf;
    uint8_t accDlpfBypass;
    uint8_t accDenom;
} gyro_device_config_t;


#define GYRO_BUFFER_SIZE 256
#define GYRO_TEMP_MULTIPLIER 0.0030599755201958f
#define GYRO_DPS_SCALE_4000 0.1219512195121951f
#define ACC_DPS_SCALE_4000 0.0009765625f
#define GYRO_DPS_SCALE_2000 0.060975609756098f
#define ACC_DPS_SCALE_2000 0.00048828125f

extern SPI_HandleTypeDef gyroSPIHandle;
extern DMA_HandleTypeDef hdmaGyroSPIRx;
extern DMA_HandleTypeDef hdmaGyroSPITx;

extern int skipGyro;

extern void gyro_init(void);
// extern void gyro_passthrough_start(void);
extern void gyro_rx_complete_callback(SPI_HandleTypeDef *hspi);
extern void gyro_exti_callback(void);