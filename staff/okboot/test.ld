__SRAM_BASE = 0x20000000;
__SRAM_SIZE = 520K;

__STACK_SIZE = 32K;

MEMORY
{
  RAM (rwx) : ORIGIN = __SRAM_BASE, LENGTH = __SRAM_SIZE
}

SECTIONS {
  .text : {
    . = ALIGN(4);
    KEEP(*(.text.start))
    *(.text*)
    . = ALIGN(4);
  } >RAM

  .data : {
    __global_pointer$ = . + 0x800;
    . = ALIGN(4);
    *(.data*)
    . = ALIGN(4);
  } >RAM

  .rodata : {
    . = ALIGN(4);
    *(.rodata*)
    . = ALIGN(4);
  } >RAM

  .bss : {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss*)
    __bss_end__ = .;
    . = ALIGN(4);
  } >RAM

  .stack (ORIGIN(RAM) + LENGTH(RAM) - __STACK_SIZE) (COPY) : {
    . = ALIGN(4);
    __stack_limit__ = .;

    . = . + __STACK_SIZE;
    . = ALIGN(4);
    __stack_top__ = .;
  } >RAM
}
