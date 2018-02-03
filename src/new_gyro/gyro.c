#include "includes.h"
#include "gyro.h"

volatile int calibratingGyro;
volatile axisData_t gyroSum;
volatile axisData_t gyroCalibrationTrim;

SPI_InitTypeDef gyroSpiInitStruct;
DMA_InitTypeDef gyroDmaInitStruct;