#pragma once
//springboard header file
#include "includes.h"


#define GYRO_SPI            SPI2
#define GYRO_CS_HARDWARE    0
#define GYRO_CS_PIN         GPIO_PIN_12
#define GYRO_CS_PORT        GPIOB
#define GYRO_CS_ALTERNATE   GPIO_AF5_SPI2
#define GYRO_MISO_PIN       GPIO_PIN_14
#define GYRO_MISO_PORT      GPIOB
#define GYRO_MISO_ALTERNATE GPIO_AF5_SPI2
#define GYRO_MOSI_PIN       GPIO_PIN_15
#define GYRO_MOSI_PORT      GPIOB
#define GYRO_MOSI_ALTERNATE GPIO_AF5_SPI2
#define GYRO_SCK_PIN        GPIO_PIN_13
#define GYRO_SCK_PORT       GPIOB
#define GYRO_SCK_ALTERNATE  GPIO_AF5_SPI2

#define BOOTLOADER_CHECK_PORT  GPIOB
#define BOOTLOADER_CHECK_PIN   GPIO_PIN_5