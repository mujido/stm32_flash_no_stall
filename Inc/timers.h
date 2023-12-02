#pragma once

#include "main.h"

extern volatile uint32_t uwTick;
extern volatile int enable_counter;
extern volatile unsigned tickcounter;
extern volatile uint32_t tick2;

static inline RAMFUNC uint32_t GetTick(void) { return uwTick; }

static inline RAMFUNC void IncTick(void) { uwTick += uwTickFreq; }

void RAMFUNC Delay(uint32_t delay);