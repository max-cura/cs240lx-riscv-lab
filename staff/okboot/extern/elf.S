@ vim:ft=arm
@
@@
@@ FILE device/okboot/extern/elf.S
@@ AUTH mcura
@@ DESC Relocatable micro-stub that performs a series of memory-to-memory ELF
@@      PT_LOAD operations.
@@
@@ CHANGELOG:
@@  31 Jan 25 (mcura)
@@      Created based on stub.S for version 2 bootloader (okboot)

#define _prefetch_flush(reg)    \
    mov reg, #0;                 \
    mcr p15, 0, reg, c7, c5, 4

#define _cln_inv_dcache_entire(reg)     \
    mov reg, #0;                         \
    mcr p15, 0, reg, c7, c14, 0

#define _inv_both_caches_entire(reg)    \
    mov reg, #0;                         \
    mcr p15, 0, reg, c7, c7, 0

#define _btac_flush(reg)        \
    mov reg, #0;                 \
    mcr p15, 0, reg, c7, c5, 6

#define _dsb(reg) \
    mov reg, #0; \
    mcr p15, 0, reg, c7, c10, 4

.globl __symbol_relocation_elf
.globl __symbol_relocation_elf_end

#define ENABLE_DEBUG 0

#if ENABLE_DEBUG
#   define putss(str) push {r0};adr r0,str;bl uart_puts;pop {r0}
#   define putc(chr) push {r0};mov r0, chr; bl uart_write8;pop {r0}
#   define putx(num) push {r0};mov r0, num; bl uart_write_u32x;pop {r0}
#else
#   define putss(x)
#   define putc(x)
#   define putx(x)
#endif

# Elf32_Phdr
#   p_type  - ignore
#   p_offset
#   p_vaddr
#   p_paddr - ignore
#   p_filesz - ignore
#   p_memsz
#   p_flags - ignore
#   p_align - ignore

#define P_OFFSET 0x4
#define P_VADDR 0x8
#define P_MEMSZ 0x14
#define PHDR_SIZE 0x20

@ r0 = pheaders : *const Elf32_Phdr
@ r1 = pheaders_count : usize
@ r2 = elf "file" base : *const u8
@ r3 = entry : word
__symbol_relocation_elf:
    mov sp, #0x8000

    putc(#'\n')
    putss(str.begin_elf_reloc)
    bl copy_segments
    putss(str.finished_copy_segments)
    bl clear_caches
    bx r3

@ r0 = pheaders
@ r1 = pheaders_count
@ r2 = elf "file" base
.align 4
copy_segments: @ COPY ELF SEGMENTS
    push {r0-r2, r4-r11, r14}
    mov r9, r0
    mov r10, r1
    mov r11, r2
.phdr_loop:
    putx(r9)
    putx(r10)
    ldr r0, [r9, #P_VADDR]
    ldr r2, [r9, #P_MEMSZ]
    movs r2, r2
    beq .phdr_fill_zero
    ldr r1, [r9, #P_OFFSET]
    add r1, r1, r11
    bl memcpy
    b .phdr_loop2
.phdr_fill_zero:
    bl memset
.phdr_loop2:
    subs r10, r10, #1
    beq .copy_segments.ret
    add r9, r9, #PHDR_SIZE
    b .phdr_loop
.copy_segments.ret:
    pop {r0-r2, r4-r11, r15}

@ CLOBBERS: r4
clear_caches:
    _dsb(r4)
    _cln_inv_dcache_entire(r4)
    _inv_both_caches_entire(r4)
    _btac_flush(r4)
    _prefetch_flush(r4)
    _dsb(r4)
    bx lr

@ --- UTILITY ROUTINES --

@ INPUT: r0 <- #0, len=r2
@ CLOBBER: r0, r4, r5, r6, r7, r8
memset:
.memset_try_coarse:
    mov r4, #0
    mvn r8, #0xf
    ands r8, r2, r8
    beq .memset_try_fine
    mov r5, #0
    mov r6, #0
    mov r7, #0
.memset_loop_coarse:
    stmia r0!, {r4, r5, r6, r7}
    subs r8, r8, #0x10
    bne .memset_loop_coarse
.memset_try_fine:
    ands r8, r2, #0x0f
    beq .memset_done
.memset_loop_fine:
    str r4, [r0], +#4
    subs r8, r8, #4
    bne .memset_loop_fine
.memset_done:
    bx lr

@ INPUT: r0 <- r1, len=r2
@ clobber: r0, r1, r4, r5, r6, r7, r8
memcpy:
.memcpy_try_coarse:
    mvn r8, #0xf
    ands r8, r2, r8
    beq .memcpy_try_fine
.memcpy_loop_coarse:
    ldmia r1!, {r4, r5, r6, r7}
    stmia r0!, {r4, r5, r6, r7}
    subs r8, r8, #0x10
    bne .memcpy_loop_coarse
.memcpy_try_fine:
    ands r8, r2, #0x0f
    beq .memcpy_done
.memcpy_loop_fine:
    ldr r4, [r1], +#4
    str r4, [r0], +#4
    subs r8, r8, #4
    bne .memcpy_loop_fine
.memcpy_done:
    bx lr

@ --- DEBUGGING ROUTINES ---

#if ENABLE_DEBUG
str.begin_elf_reloc: .asciz "[asm] beginning ELF relocation"
str.finished_copy_segments: .asciz "[asm] finished copying segments"
#else
str.begin_elf_reloc:
str.finished_copy_segments:
#endif

#if ENABLE_DEBUG

.parled.on:
    push {r4, r10, r14}
    _dsb(r4)
    ldr r10, loc.gpio_base
    mov r4, #(1 << 27)
    str r4, [r10, #0x1c]
    _dsb(r4)
    pop {r4, r10, r15}
.parled.off:
    push {r4, r10, r14}
    _dsb(r4)
    ldr r10, loc.gpio_base
    mov r4, #(1 << 27)
    str r4, [r10, #0x28]
    _dsb(r4)
    pop {r4, r10, r15}
.actled.on:
    push {r4, r10, r14}
    _dsb(r4)
    ldr r10, loc.gpio_base
    mov r4, #1
    lsl r4, r4, #(47-32)
    str r4, [r10, #0x2c]
    _dsb(r4)
    pop {r4, r10, r15}
.actled.off:
    push {r4, r10, r14}
    _dsb(r4)
    ldr r10, loc.gpio_base
    mov r4, #1
    lsl r4, r4, #(47-32)
    str r4, [r10, #0x20]
    _dsb(r4)
    pop {r4, r10, r15}

loc.gpio_base: .word 0x20200000

#define MU_STAT 0x64
#define MU_STAT_TX_READY 1
#define MU_LSR 0x54
#define MU_LSR_FIFO_CAN_WRITE 5
#define MU_IO 0x40

uart_puts:
    push {r0, r1, lr}
    mov r1, r0
.ascii_chars:
    ldrb r0, [r1], +#1
    cmp r0, #0
    beq .newline_puts
    bl uart_write8
    b .ascii_chars
.newline_puts:
    mov r0, #'\n'
    bl uart_write8
    pop {r0, r1, pc}

uart_write_u32x:
    push {r0, r1, r2, r3, lr}
    mov r3, #0xf
    mov r2, r0
    mov r1, #32
.digits:
    sub r1, r1, #4
    and r0, r2, r3, lsl r1
    mov r0, r0, lsr r1
    subs r0, r0, #10
    addlt r0, r0, #('0' + 10)
    addge r0, r0, #'a'
    bl uart_write8
    cmp r1, #0
    beq .newline32
    b .digits
.newline32:
    mov r0, #'\n'
    bl uart_write8
    pop {r0, r1, r2, r3, pc}

uart_write8:
    push {r1, r2, r14}
    _dsb(r1)
    ldr r2, loc.uart_base
.uart_busy:
    ldr r1, [r2, #MU_LSR]
    tst r1, #(1 << MU_LSR_FIFO_CAN_WRITE)
    bne .uart_can_write @ if bit is set
    b .uart_busy
.uart_can_write:
    str r0, [r2, #MU_IO]
    _dsb(r1)
    pop {r1, r2, r15}

loc.uart_base: .word 0x20215000

#endif

@ --- TERMINATING SYMBOL ---

__symbol_relocation_elf_end:
    nop
    nop
