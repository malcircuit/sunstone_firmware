/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys -lm)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x18000, LENGTH = 0x28000
  RAM (rwx) :  ORIGIN = 0x20002000, LENGTH = 0x2000
}

INCLUDE "gcc_nrf51_common.ld"