#pragma once
#include "includes.h"

#define GYRO_TEMP_MULTIPLIER 0.0030599755201958f
#define GYRO_DPS_SCALE_4000 0.1219512195121951f
#define ACC_DPS_SCALE_4000 0.0009765625f
#define GYRO_DPS_SCALE_2000 0.060975609756098f
#define ACC_DPS_SCALE_2000 0.00048828125f

#define CALIBRATION_CYCLES 1900
#define GYRO_BUFFER_SIZE 256

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

typedef void (*gyro_read_done_t)(gyroFrame_t* gyroRxFrame);

extern gyroFrame_t gyroRxFrame;

extern const gyro_device_config_t gyroConfig;
extern float gyroRateMultiplier;
extern float gyroAccMultiplier;
extern void gyro_device_init(gyro_read_done_t doneFn);