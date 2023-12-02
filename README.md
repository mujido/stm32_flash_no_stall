# Erase Flash without blocking execution on Cortex-M0

This is an example based loosely on the STM32 HAL flash Erase Pages example.
Flash erase normally will block execution of flash-based code during erase
cycle. One can run directly from RAM to avoid this. 

For this reason, I wanted to try to make a hybrid firmware where some code would
run from flash primarily but run from RAM the sections required during the flash
erase cycles. The basic sequence is system initialization, call DoErase() where
all this code and its dependencies run from SRAM and therefore don't block.
After return from DoErase(), the code continues executing from flash. 

Notably the SysTick and TIM2 timer IRQ handlers also run from SRAM from
initialization as well. This is demonstrated by LED4 (Orange LED on STM
32F072BDISCOVERY) which flashes at a 50kHz rate. When I had a bug with something
using flash, the LED would flash eradically. After fixing the LED remains at a
constantish flash.

Moving functions to SRAM is fairly well documented online. The gist involves
adding a RamFunc section to linker script. I added these as so to the data
section.

```
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    
    . = ALIGN(4);
    *(.RamFunc)
    *(.RamFunc*)
    
    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH
```

By adding it to the `.data` section, we automatically copy the functions to SRAM
at startup using the normal data initialization startup code.

Unlike some easier to use chips, the Cortex-M0 series do not have a relocatable
ISR vector via a mechanism like `VTOR`. It does however have a means of
remapping the base of memory (`0x0000_0000`) to one of flash, SRAM, or system
memory. This project uses this mechanism, via `SYSCFG->CFGR1` `MEM_MODE` bits.
In the `SystemInit()` function, we copy the ISR vector to a section of RAM
reserved via the linker file which must be placed at exactly the start of SRAM,
`0x2000_0000`. This is so that after remap the ISR will still be placed at
`0x0000_0000`. Here is the relevant section of linker script to accomplish it.

```
  .ram_isr_vector 0x20000000 :
  {
    ram_isr_vector = .;
    . += SIZEOF(.isr_vector);
    ram_isr_vector_end = .;
  } >RAM
```