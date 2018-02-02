#include "includes.h"

//volatile uint32_t millisClock = 0;
//volatile uint32_t systemUsTicks;
volatile uint32_t ticks = 0;

static void sys_tick_config(void);

uint32_t millis(void)
{
    return ticks;
}

static void sys_tick_config(void)
{

  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Setup SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 100))
  {
    /* Capture error */
    while (1);
  }

  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}

void clock_config(void)
{
    //TODO: Will need to add all the clocks
    //Enable SPI clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    //enable DMA clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    //enable GPIO clocks
    RCC_AHBPeriphClockCmd(
        RCC_AHBPeriph_GPIOA |
        RCC_AHBPeriph_GPIOB |
        RCC_AHBPeriph_GPIOC |
        RCC_AHBPeriph_GPIOD |
        RCC_AHBPeriph_GPIOE
        , ENABLE
    );

    //enable SYSCFG clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}


void update_millis_clock(void)
{
    //millisClock = DWT->CYCCNT; //used for micros seconds if we need it
    ticks += 10; //10 ms ticks
}