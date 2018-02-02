#include "includes.h"

volatile uint32_t millisClock = 0;
volatile uint32_t systemUsTicks;
volatile uint32_t ticks = 0;

static void sys_tick_config(void);

void delay_ms(uint32_t ms)
{
    while (ms-- > 0)
    {
        delay_us(1000);
    }
}

void delay_us(uint32_t us)
{
    volatile uint32_t DWT_START = DWT->CYCCNT;
    volatile uint32_t DWT_TOTAL = (systemUsTicks * us);

    while ( (DWT->CYCCNT - DWT_START) < DWT_TOTAL);
}

uint32_t millis(void)
{
    return ticks;
}

uint32_t micros(void)
{

	volatile uint32_t baseMillis;
	volatile uint32_t baseClock;

    int is = __get_PRIMASK();
    __disable_irq();

    baseMillis = millis();
    baseClock = millisClock;

    uint32_t elapsedSinceMillis = ( (DWT->CYCCNT-baseClock) / systemUsTicks );

    if ((is & 1) == 0)
	{
        __enable_irq();
    }

    return ((baseMillis * 1000) + elapsedSinceMillis);

}

static void sys_tick_config(void)
{

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Setup SysTick Timer for 1ms interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        /* Capture error */
        while (1);
    }

    systemUsTicks = (SystemCoreClock/1000000);   
    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

void clock_config(void)
{
    //TODO: Will need to add all the clocks
    //Enable SPI clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    RCC_I2SCLKConfig(RCC_I2S2CLKSource_SYSCLK);

    //enable DMA clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    //enable GPIO clocks
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOA |
        RCC_AHBPeriph_GPIOB |
        RCC_AHBPeriph_GPIOC |
        RCC_AHBPeriph_GPIOD |
        RCC_AHBPeriph_GPIOE |
        RCC_AHBPeriph_GPIOF
        , ENABLE
    );

    //enable SYSCFG clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    sys_tick_config();
}


void update_millis_clock(void)
{
    ticks++; //10 ms ticks
    millisClock = DWT->CYCCNT;
}