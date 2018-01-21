#pragma once

typedef struct gyro_data {
    uint8_t rateDiv;
    uint8_t gyroDlpf;
    uint8_t gyroDlpfBypass;
    uint8_t accDlpf;
    uint8_t accDlpfBypass;
    uint8_t accDenom;
} gyro_device_config_t;

extern SPI_HandleTypeDef gyroSPIHandle;
extern DMA_HandleTypeDef hdmaGyroSPIRx;
extern DMA_HandleTypeDef hdmaGyroSPITx;
extern uint8_t gyroSpiRxBuffer[];
extern uint8_t gyroSpiTxBuffer[];

extern gyro_device_config_t gyroConfig;
extern int skipGyro;

extern void gyro_init(void);
extern void gyro_passthrough_start(void);
extern void gyro_rx_complete_callback(SPI_HandleTypeDef *hspi);
extern void gyro_exti_callback(void);