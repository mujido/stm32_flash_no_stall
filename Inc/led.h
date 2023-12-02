#pragma once

#include "main.h"
#include "my_gpio.h"

#define LEDn 4

#define LED3_PIN                         GPIO_PIN_6
#define LED3_GPIO_PORT                   GPIOC
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE() 

#define LED4_PIN                         GPIO_PIN_8
#define LED4_GPIO_PORT                   GPIOC
#define LED4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define LED4_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE() 
  
#define LED5_PIN                         GPIO_PIN_9
#define LED5_GPIO_PORT                   GPIOC
#define LED5_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define LED5_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()  

#define LED6_PIN                         GPIO_PIN_7
#define LED6_GPIO_PORT                   GPIOC
#define LED6_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define LED6_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE() 

#define LEDx_GPIO_CLK_ENABLE(__LED__) do { if((__LED__) == LED3) LED3_GPIO_CLK_ENABLE(); else \
                                           if((__LED__) == LED4) LED4_GPIO_CLK_ENABLE(); else \
                                           if((__LED__) == LED5) LED5_GPIO_CLK_ENABLE(); else \
                                           if((__LED__) == LED6) LED6_GPIO_CLK_ENABLE();} while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)  (((__LED__) == LED3) ? LED3_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED4) ? LED4_GPIO_CLK_DISABLE() :\
                                         ((__LED__) == LED5) ? LED5_GPIO_CLK_DISABLE() : \
                                         ((__LED__) == LED6) ? LED6_GPIO_CLK_DISABLE() : 0 )
typedef enum 
{
    LED3 = 0,
    LED4 = 1,
    LED5 = 2,
    LED6 = 3,
    /* Color led aliases */
    LED_RED    = LED3,
    LED_ORANGE = LED4,
    LED_GREEN  = LED5,
    LED_BLUE   = LED6
} Led_TypeDef;

extern GPIO_TypeDef* My_LED_PORT[LEDn];
extern uint16_t My_LED_PIN[LEDn];

static inline void RAMFUNC My_LED_On(Led_TypeDef Led)
{
  My_GPIO_WritePin(My_LED_PORT[Led], My_LED_PIN[Led], GPIO_PIN_SET);
}


static inline void RAMFUNC My_LED_Off(Led_TypeDef Led)
{
  My_GPIO_WritePin(My_LED_PORT[Led], My_LED_PIN[Led], GPIO_PIN_RESET);
}

static inline void RAMFUNC My_LED_Toggle(Led_TypeDef Led)
{
  My_GPIO_TogglePin(My_LED_PORT[Led], My_LED_PIN[Led]);
}

void My_LED_Init(Led_TypeDef Led);
