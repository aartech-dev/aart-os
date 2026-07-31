MEMORY
{
  /* Nucleo-G474RE = STM32G474RET6: 512 KB flash, 128 KB SRAM.
     Retargeted from the G431RB (128K/32K) to pick up ADC3/ADC4 for
     dedicated per-motor neutral sensing - see DESIGN.md section 6.1. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
