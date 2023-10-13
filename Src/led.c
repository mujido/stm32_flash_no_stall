
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
