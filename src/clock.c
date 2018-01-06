#include "includes.h"

volatile uint32_t millisClock = 0;
volatile uint32_t systemUsTicks;

void init_clk(void) 
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType      = RCC_ALL_CLK;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void init_osc(void) 
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
}

void SystemClock_Config(void)
{
  init_osc();
  init_clk();
	/* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  systemUsTicks = (HAL_RCC_GetHCLKFreq()/1000000);
}


void UpdateMillisClock(void)
{
	millisClock = DWT->CYCCNT;
}