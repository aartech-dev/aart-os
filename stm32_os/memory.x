MEMORY
{
  /* Nucleo-G431RB = STM32G431RBT6: 128 KB flash, 32 KB SRAM.
     The previous value (64K flash) silently threw away half the part. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
