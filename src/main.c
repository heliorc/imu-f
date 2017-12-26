#include "includes.h"


void InitializeGpio(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t on)
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

void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  systemUsTicks = (HAL_RCC_GetHCLKFreq()/1000000);
}

int BoardInit(void)
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


int main(void) 
{
    volatile int mouse = 0;
    BoardInit();

    InitializeGpio(GPIOB, GPIO_PIN_5, 1);

    while(1)
    {
        GPIOB->ODR = 0xFFFF;
        //InlineDigitalHi(GPIOB, GPIO_PIN_5);
        //mouse++;
        GPIOB->ODR = 0x0000;
        //InlineDigitalLo(GPIOB, GPIO_PIN_5);
        //mouse++;
    }

    return(0);
}