#pragma once

typedef struct gyro_data {
    uint8_t rateDiv;
    uint8_t gyroDlpf;
    uint8_t gyroDlpfBypass;
    uint8_t accDlpf;
    uint8_t accDlpfBypass;
    uint8_t accDenom;
} gyro_device_config_t;

extern gyro_device_config_t gyroConfig;

extern void gyro_init(void);
extern void gyro_passthrough_start(void);
extern void gyro_rx_complete_callback(SPI_HandleTypeDef *hspi);