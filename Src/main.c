/**
 ******************************************************************************
 * @file    FLASH/FLASH_EraseProgram/Src/main.c
 * @author  MCD Application Team
 * @brief   This example provides a description of how to erase and program the
 *          STM32F0xx FLASH.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define RAMFUNC __attribute__((section(".RamFunc")))
#define NOINLINE __attribute__((noinline))
#define FLASHAPI RAMFUNC

#define __FLASH_GET_FLAG(__FLAG__) (((FLASH->SR) & (__FLAG__)) == (__FLAG__))
#define __FLASH_CLEAR_FLAG(__FLAG__)   ((FLASH->SR) = (__FLAG__))

#define FLASH_USER_START_ADDR ADDR_FLASH_PAGE_16                 /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR ADDR_FLASH_PAGE_63 + FLASH_PAGE_SIZE /* End @ of user Flash area */

#define DATA_32 ((uint32_t)0x12345678)

#define My_LED_Init BSP_LED_Init

volatile uint32_t myTicks = 0;
volatile int enable_counter = 0;
volatile unsigned tickcounter = 0;
volatile uint32_t tick2 = 0;

uint32_t Address = 0, PageError = 0;
__IO uint32_t data32 = 0, MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

extern volatile int enable_counter;
extern volatile unsigned tickcounter;

static GPIO_TypeDef* My_LED_PORT[LEDn] = {LED3_GPIO_PORT,
                                LED4_GPIO_PORT,
                                LED5_GPIO_PORT,
                                LED6_GPIO_PORT};

static const uint16_t My_LED_PIN[LEDn] = {LED3_PIN,
                                LED4_PIN,
                                LED5_PIN,
                                LED6_PIN};



static RAMFUNC uint32_t GetTick(void)
{
    return myTicks;
}

static void RAMFUNC Delay(uint32_t delay)
{
  uint32_t tickstart = GetTick();
  uint32_t wait = delay;
  
  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }
  
  while((GetTick() - tickstart) < wait)
  {
  }
}

void RAMFUNC My_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = (uint32_t)GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = (uint32_t)GPIO_Pin;
  }
}

void RAMFUNC My_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint32_t odr;

  /* get current Output Data Register value */
  odr = GPIOx->ODR;

  /* Set selected pins that were at low level, and reset ones that were high */
  GPIOx->BSRR = ((odr & GPIO_Pin) << 16u) | (~odr & GPIO_Pin);
}

void RAMFUNC My_LED_On(Led_TypeDef Led)
{
  My_GPIO_WritePin(My_LED_PORT[Led], My_LED_PIN[Led], GPIO_PIN_SET);
}


void RAMFUNC My_LED_Off(Led_TypeDef Led)
{
  My_GPIO_WritePin(My_LED_PORT[Led], My_LED_PIN[Led], GPIO_PIN_RESET);
}

void RAMFUNC My_LED_Toggle(Led_TypeDef Led)
{
  My_GPIO_TogglePin(My_LED_PORT[Led], My_LED_PIN[Led]);
}

HAL_StatusTypeDef FLASHAPI My_FLASH_Unlock(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);

    /* Verify Flash is unlocked */
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
      status = HAL_ERROR;
    }
  }

  return status;
}

static int FLASHAPI WaitForLastFlashOperation(uint32_t Timeout)
{
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
       Even if the FLASH operation fails, the BUSY flag will be reset and an error
       flag will be set */

    uint32_t tickstart = GetTick();

    while (__FLASH_GET_FLAG(FLASH_FLAG_BSY))
    {
        if ((Timeout == 0U) || ((GetTick() - tickstart) > Timeout))
        {
            return 0;
        }
    }

    /* Check FLASH End of Operation flag  */
    if (__FLASH_GET_FLAG(FLASH_FLAG_EOP))
    {
        /* Clear FLASH End of Operation pending bit */
        __FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    }

    if (__FLASH_GET_FLAG(FLASH_FLAG_WRPERR) ||
        __FLASH_GET_FLAG(FLASH_FLAG_PGERR))
    {
        return 0;
    }

    /* There is no error flag set */
    return 1;
}

static void NOINLINE FLASHAPI My_PageErase(uint32_t PageAddress)
{
    /* Proceed to erase the page */
	My_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    SET_BIT(FLASH->CR, FLASH_CR_PER);

	My_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    WRITE_REG(FLASH->AR, PageAddress);

	My_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);
}

static int NOINLINE FLASHAPI EraseFlash(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError)
{
	int status = 1;

    /* Page Erase requested on address located on bank1 */
    /* Wait for last operation to be completed */
    if (WaitForLastFlashOperation((uint32_t)FLASH_TIMEOUT_VALUE))
    {
        /*Initialization of PageError variable*/
        *PageError = 0xFFFFFFFFU;

        /* Erase page by page to be done*/
        for (uint32_t address = pEraseInit->PageAddress;
             address < ((pEraseInit->NbPages * FLASH_PAGE_SIZE) + pEraseInit->PageAddress);
             address += FLASH_PAGE_SIZE)
        {
            My_PageErase(address);

            /* Wait for last operation to be completed */
            status = WaitForLastFlashOperation((uint32_t)FLASH_TIMEOUT_VALUE);

            /* If the erase operation is completed, disable the PER Bit */
            CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

            if (!status)
            {
                /* In case of error, stop erase procedure and return the faulty address */
                *PageError = address;
                break;
            }
        }
    }

    return status;
}

static int NOINLINE RAMFUNC DoErase(void)
{
	/* Unlock the Flash to enable the flash control register access *************/
    My_FLASH_Unlock();

    /* Erase the user Flash area
      (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

    My_LED_Off(LED6);
    My_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    enable_counter = 1;
    int rc = EraseFlash(&EraseInitStruct, &PageError);
    enable_counter = 0;

    My_LED_Off(LED_GREEN);

    unsigned blinks = tickcounter / 8;
    for (unsigned i = 0; i < blinks; ++i)
    {
        My_LED_On(LED_GREEN);
        Delay(350);
        My_LED_Off(LED_GREEN);
        Delay(350);
    }

    return rc;
}

static void Setup_Timer2(void);

/**
 * @brief  Main program
 * @param  None
 * @retval None
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

    if (!DoErase())
    {
        /*
          Error occurred while page erase.
          User can add here some code to deal with this error.
          PageError will contain the faulty page and then to know the code error on this page,
          user can call function 'HAL_FLASH_GetError()'
        */
        /* Infinite loop */
        while (1)
        {
            My_LED_On(LED5);
        }
    }

	/* No error detected. Switch on LED3*/
	My_LED_On(LED3);

	/* Infinite loop */
    while (1)
    {
    }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSI48)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 48000000
 *            PREDIV                         = 2
 *            PLLMUL                         = 2
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
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


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
    /* Turn LED5 on */
    My_LED_On(LED5);
    while (1)
    {
    }
}

/**
 * @}
 */

/**
 * @}
 */

int _close(int)
{
    return 0;
}

int _lseek()
{
    return 0;
}

int _read()
{
    return 0;
}

int _write()
{
    return 0;
}

void RAMFUNC NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void RAMFUNC HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void RAMFUNC SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void RAMFUNC PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void RAMFUNC SysTick_Handler(void)
{
    uwTick += uwTickFreq;
    myTicks = uwTick;

    // HAL_IncTick();

    if (enable_counter) // && uwTick % 10 == 0)
    {
    	tickcounter++;
#if 1
    	My_LED_Toggle(LED5);
#endif
    }
}

void RAMFUNC TIM2_IRQHandler(void)
{
	TIM2->SR &= ~TIM_SR_UIF;

	tick2++;
	My_LED_Toggle(LED4);

	TIM2->CCMR1 ^= (1U << TIM_CCMR1_OC1M_Pos);
}

void RAMFUNC DefaultHandler(void)
{
	while(1);
}

