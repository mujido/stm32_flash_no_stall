/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "flash.h"
#include "timers.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


static void Setup_Timer2(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* STM32F0xx HAL library initialization:
         - Configure the Flash prefetch
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Low Level Initialization
       */
    HAL_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Initialize LED3, LED4 and LED5 */
    My_LED_Init(LED3);
    My_LED_Init(LED4);
    My_LED_Init(LED5);
    My_LED_Init(LED6);

    /* Configure the system clock to 48 MHz */
    SystemClock_Config();
    Setup_Timer2();

    My_LED_On(LED6);

    GPIO_InitTypeDef pa5_init =
    {
    	.Pin = GPIO_PIN_5,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_HIGH,
		.Alternate = GPIO_AF2_TIM2,
    };
    HAL_GPIO_Init(GPIOA, &pa5_init);

    GPIO_InitTypeDef pa15_init =
    {
    	.Pin = GPIO_PIN_15,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(GPIOA, &pa15_init);
    My_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    GPIO_InitTypeDef pb15_init =
    {
    	.Pin = GPIO_PIN_15,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(GPIOB, &pb15_init);
    My_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_Delay(1000);

    TIM2->CCMR1 = (0b100 << TIM_CCMR1_OC1M_Pos);

    if (My_OK != DoErase())
    {
      while (1)
      {
        My_LED_Off(LED_RED);
        Delay(200);
        My_LED_On(LED_RED);
        Delay(200);
      }
    }

	/* No error detected. */
	My_LED_On(LED_RED);

	/* Infinite loop */
    while (1)
    {
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Select HSI48 Oscillator as PLL source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }

    DBGMCU->APB1FZ = DBGMCU_APB1_FZ_DBG_TIM2_STOP;
}

static void Setup_Timer2(void)
{
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* (1) Enable the peripheral clock of Timer 2 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->ARR = HAL_RCC_GetSysClockFreq() / 100000;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->EGR = 0;

	// Setup CH1 for Capture Output Mode
	TIM2->CCR1 = UINT32_MAX;
	TIM2->CCMR1 = (0b011 << TIM_CCMR1_OC1M_Pos);
	TIM2->CCER = TIM_CCER_CC1E;

	// Start TIM2
	TIM2->CR1 |= TIM_CR1_URS | TIM_CR1_CEN;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
