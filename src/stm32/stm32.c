#include "includes.h"

//functions specific to this particular MCU Family

typedef void (*pFunction)(void);

static volatile uint32_t millisClock = 0;
static volatile uint32_t usbStarted = 0;

extern void UsbDeviceInit(void);
extern void UsbDeviceDeInit(void);

volatile uint32_t systemUsTicks;

uint32_t InlineMillis(void)
{
	return(HAL_GetTick());
}

void InlineUpdateMillisClock(void)
{
	millisClock = DWT->CYCCNT;
}

uint32_t InlineMicros(void)
{

	volatile uint32_t baseMillis;
	volatile uint32_t baseClock;

    int is = __get_PRIMASK();
    __disable_irq();

    baseMillis = InlineMillis();
    baseClock = millisClock;

    uint32_t elapsedSinceMillis = ( (DWT->CYCCNT-baseClock) / systemUsTicks );

    if ((is & 1) == 0)
	{
        __enable_irq();
    }

    return((baseMillis * 1000) + elapsedSinceMillis);

}

inline void InlineDelayUs(uint32_t uSec)
{
    volatile uint32_t DWT_START = DWT->CYCCNT;
    volatile uint32_t DWT_TOTAL = (systemUsTicks * uSec);
    while( (DWT->CYCCNT - DWT_START) < DWT_TOTAL);
}

inline void InlineDelayMs(uint32_t mSec)
{
	HAL_Delay(mSec);
}


inline void inline_digital_hi(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

inline void inline_digital_lo(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

inline int InlineIsPinStatusHi(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
    {
		return 0; //pin is set, so it is not reset, which means it is off, so the statement is false
	}
	return 1; //pin is reset, so it is not set, which means it is on, so the statement is true
}

void DeInitGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_DeInit(GPIOx, GPIO_Pin);
}

void gpio_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t on)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

    if (on) {
    	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    } else {
    	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    }

    GPIO_InitStructure.Pin   = GPIO_Pin;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);

}

void init_gpio_input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t GPIO_Pull)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

    GPIO_InitStructure.Pin   = GPIO_Pin;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull  = GPIO_Pull;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);
}