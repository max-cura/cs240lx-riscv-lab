__XIP_BASE = 0x10000000;
__XIP_SIZE = 32M;

__SRAM_BASE = 0x20000000;
__SRAM_SIZE = 520K;

__STACK_SIZE = 32K;

MEMORY
{
    RAM (rwx) : ORIGIN = __SRAM_BASE, LENGTH = __SRAM_SIZE
    FLASH (rx): ORIGIN = __XIP_BASE,  LENGTH = __XIP_SIZE
}

PHDRS
{
    text PT_LOAD FLAGS(5);
    data PT_LOAD FLAGS(6);
    bss PT_LOAD FLAGS(6);
}

SECTIONS
{
    .text : {
        . = ALIGN(4);
        KEEP(*(.rp2350.boot_jump))
        KEEP(*(.rp2350.boot_header))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH :text

    .data : {
        __global_pointer$ = . + 0x800;
        . = ALIGN(4);
        __data_start__ = .;
        *(.data*)
        . = ALIGN(4);
        __data_end__ = .;
    } >RAM AT> FLASH :data

    .bss : {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        . = ALIGN(4);
        __bss_end__ = .;
    } >RAM :bss

    .stack (ORIGIN(RAM) + LENGTH(RAM) - __STACK_SIZE) (COPY) : {
        . = ALIGN(4);
        __stack_limit__ = .;

        . = . + __STACK_SIZE;
        . = ALIGN(4);
        __stack_top__ = .;
    } >RAM
}
