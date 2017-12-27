#include "includes.h"

int stm32_init(void)
{
	SCB->VTOR = 0x08000000; //set vector register to firmware start
	__enable_irq(); // enable interrupts

	HAL_Init();

	SystemClock_Config();

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();


  __HAL_RCC_DMA1_CLK_ENABLE();
  /*
  __USART1_CLK_ENABLE();
  __USART2_CLK_ENABLE();
  __USART3_CLK_ENABLE();
  __UART4_CLK_ENABLE();
  __UART5_CLK_ENABLE();
  __USART6_CLK_ENABLE();
  */
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM6_CLK_ENABLE();

  return(0);
}