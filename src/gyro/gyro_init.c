#include "includes.h"


static void spi_gpio_init(void);
static void spi_init(uint32_t baudRatePrescaler);

SPI_HandleTypeDef gyroSpiHandle;

static void spi_gpio_init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_GPIO_DeInit(GYRO_CS_PORT,  GYRO_CS_PIN);
    HAL_GPIO_DeInit(GYRO_MISO_PORT,  GYRO_MISO_PIN);
    HAL_GPIO_DeInit(GYRO_MOSI_PORT,  GYRO_MOSI_PIN);
    HAL_GPIO_DeInit(GYRO_SCK_PORT,  GYRO_SCK_PIN);

	GPIO_InitStruct.Pin   = GYRO_CS_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if(GYRO_CS_HARDWARE)
        GPIO_InitStruct.Alternate = GYRO_CS_ALTERNATE;
	HAL_GPIO_Init(GYRO_CS_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin   = GYRO_SCK_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GYRO_SCK_ALTERNATE;
	HAL_GPIO_Init(GYRO_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GYRO_MISO_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GYRO_MISO_ALTERNATE;
	HAL_GPIO_Init(GYRO_MISO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GYRO_MOSI_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GYRO_MOSI_ALTERNATE;
	HAL_GPIO_Init(GYRO_MOSI_PORT, &GPIO_InitStruct);

    if(!GYRO_CS_HARDWARE)
        inline_digital_hi(GYRO_CS_PORT, GYRO_CS_PIN);

}

static void spi_init(uint32_t baudRatePrescaler)
{

    SpiHandle.Instance               = GYRO_SPI;
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.BaudRatePrescaler = baudRatePrescaler;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    if(GYRO_CS_HARDWARE)
        SpiHandle.Init.NSS               = SPI_NSS_HARD_OUTPUT;
    else
        SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;

    HAL_SPI_DeInit(&SpiHandle);

    if (HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
        while(1);
    }

    inline_digital_hi(ports[board.gyros[0].csPort], board.gyros[0].csPin);
}

void gyro_configure(void)
{

}

void gyro_exti_init(void)
{

}

int gyro_init(void)
{
    spi_gpio_init();
    spi_init(SPI_BAUDRATEPRESCALER_32);
    gyro_configure();
    spi_init(SPI_BAUDRATEPRESCALER_2);
    gyro_exti_init();
}