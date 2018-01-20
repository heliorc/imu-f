#pragma once

extern void gyro_init(void);
extern void gyro_passthrough_start(void);
extern void gyro_rx_complete_callback(SPI_HandleTypeDef *hspi);