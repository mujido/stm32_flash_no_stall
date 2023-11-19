
#include "led.h"
#include "stm32f0xx_hal.h"

GPIO_TypeDef* My_LED_PORT[LEDn] = {LED3_GPIO_PORT,
                                LED4_GPIO_PORT,
                                LED5_GPIO_PORT,
                                LED6_GPIO_PORT};

const uint16_t My_LED_PIN[LEDn] = {LED3_PIN,
                                LED4_PIN,
                                LED5_PIN,
                                LED6_PIN};

void My_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = My_LED_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP; /* GPIO_PuPd_DOWN */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(My_LED_PORT[Led], &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(My_LED_PORT[Led], My_LED_PIN[Led], GPIO_PIN_RESET); 
}
