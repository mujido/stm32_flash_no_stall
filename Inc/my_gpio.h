#pragma once

#include "main.h"

void RAMFUNC My_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void RAMFUNC My_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);