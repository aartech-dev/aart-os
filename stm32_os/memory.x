MEMORY
{
  /* Nucleo-G474RE = STM32G474RET6: 512 KB flash, 128 KB SRAM.
     Retargeted from the G431RB (128K/32K) to pick up ADC3/ADC4 for
     dedicated per-motor neutral sensing - see DESIGN.md section 6.1.

     FLASH is 2K shorter than the real 512K here on purpose: the last
     2K page (a whole erase-granularity page on this part, at
     0x0807F800-0x0807FFFF) is reserved for aart_core::params persistence
     (config_store.rs) - shrinking the linker's view of FLASH guarantees
     the linker can never place code/data there, regardless of how big
     this firmware grows, without needing a separate memory region/
     section for it. See DESIGN.md section 7.7. */
  FLASH : ORIGIN = 0x08000000, LENGTH = 510K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
