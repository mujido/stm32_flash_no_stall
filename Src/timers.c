#include "timers.h"

volatile int enable_counter = 0;
volatile unsigned tickcounter = 0;
volatile uint32_t tick2 = 0;

void RAMFUNC Delay(uint32_t delay)
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
