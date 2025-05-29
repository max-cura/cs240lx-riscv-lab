@ vim:ft=arm
@
@@
@@ FILE device/okboot/extern/stub.S
@@ AUTH mcura
@@ DESC Relocatable micro-stub that performs a `memcpy` and then jumps to an
@@      address. Part of the `theseus-device` crate. Primary symbol is
@@      `_relocation_stub`; relocatable segment is
@@
@@               [_relocation_stub, _relocation_stub_end)
@@
@@      Note that this has some additional functionality beyond merely a
@@      `memcpy` and branch; read the source for additional details (any
@@      description here will almost inevitably become de-synchronised with the
@@      code
@@
@@ CHANGELOG:
@@  15 Apr 24 (mcura)
@@      Separated from boot.S in an effort to clean up code for debugging.
@@  17 Jun 24 (mcura)
@@      More cleanup in preparation for bootloader v2

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

.globl __symbol_relocation_stub
.globl __symbol_relocation_stub_end


@
@ HACKING NOTES
@   1.  DO NOT MAKE ANY REFERENCE TO ANY SYMBOL NOT CONTAINED BETWEEN
@       _relocation_stub AND _relocation_stub_end! DOING SO MAY CAUSE DATA
@       CORRUPTION.
@       
@       Note that this includes as well the use of `ldr {reg}, ={value}`, even
@       if `value` is not explicitly a symbol (e.g. `ldr r0, =0x8000` and
@       `ldr r0, =non_local_symbol` are both equally forbidden). This is
@       because `ldr {reg}, ={value}` will attempt to resolve to an immediate
@       `MOV`, and, failing that, will put `{value}` in the local data pool.
@       I (mcura) am personally unclear on the rules of placement for these
@       values, but prior experience (via disassembly) has shown the values
@       showing up after the end of the relocation boundary--and thus not being
@       moved by the stub.
@

@ --------------------------------------------------------------------------------------------------
@ SECTION: RELOCATION
@ --------------------------------------------------------------------------------------------------

@ INPUT: r0=dest r1=src r2=len r3=jump_to
__symbol_relocation_stub:

@ INPUT: r0, r1, r2
@ OUTPUT: N/A
@ CLOBBERS: r0, r1, r4, r5, r6, r7, r8
.fastreloc_copy:
    @bl .actled.on
.fastreloc_try_coarse:
    mvn r8, #0x0f
    ands r8, r2, r8
    beq .fastreloc_try_fine
.fastreloc_loop_coarse:
    @ need ! to write the incremented value back to r1/r0 when the instruction
    @ finishes
    ldmia r1!, {r4, r5, r6, r7}
    stmia r0!, {r4, r5, r6, r7}
    subs r8, r8, #0x10
    bne .fastreloc_loop_coarse
.fastreloc_try_fine:
    ands r8, r2, #0x0f
    beq .fastreloc_done
.fastreloc_loop_fine:
    ldr r4, [r1], +#4
    str r4, [r0], +#4
    subs r8, r8, #4
    bne .fastreloc_loop_fine
.fastreloc_done:
    @bl .actled.off
    b .postreloc

@ INPUT: r0, r1, r2
@ OUTPUT: N/A
@ CLOBBERS: r0, r1, r2, r4
.slowreloc_copy:
    @bl .actled.on

    teq r2, #0
    beq .slowreloc_done
.slowreloc_loop:
    ldr r4, [r1], +#4
    str r4, [r0], +#4
    subs r2, r2, #4
    bne .slowreloc_loop
.slowreloc_done:
    b .postreloc

.postreloc:

@ INPUT: N/A
@ OUTPUT: N/A
@ CLOBBERS: r4
.clear_caches:
    _dsb(r4)

    @ not 
    _cln_inv_dcache_entire(r4)
    _inv_both_caches_entire(r4)
    _btac_flush(r4)
    _prefetch_flush(r4)

    _dsb(r4)

.jump_to_loaded_program:
    bx r3

@ --------------------------------------------------------------------------------------------------
@ SECTION: GPIO
@ --------------------------------------------------------------------------------------------------

@ INPUT: r10
@ OUTPUT: N/A
@ CLOBBERS: r4
.actled.on:
    _dsb(r4)

    mov r4, #1
    lsl r4, r4, #(47-32)
    str r4, [r10, #0x2c]

    _dsb(r4)
    
    bx lr
.actled.off:
    _dsb(r4)

    mov r4, #1
    lsl r4, r4, #(47-32)
    str r4, [r10, #0x20]

    _dsb(r4)

    bx lr

@ INPUT: N/A
@ OUTPUT: r10=loc.gpio_base
@ CLOBBERS: r4, r5, r7, r10
.gpio_setup:
    _dsb(r4)

    ldr r10, loc.gpio_base

#if 0
    @ pin 27 output (activity LED on parthiv-pi hat)
    ldr r5, [r10, #0x08]
    mov r7, #7
    mvn r7, r7, lsl #21
    and r5, r5, r7
    mov r7, #1
    orr r5, r5, r7, lsl #21
    str r5, [r10, #0x08]
    @ pin 47 output (activity LED on Pi board)
    ldr r5, [r10, #0x10]
    mov r7, #7
    mvn r7, r7, lsl #21
    and r5, r5, r7
    mov r7, #1
    orr r5, r5, r7, lsl #21
    str r5, [r10, #0x10]

    @mov r4, #1
    @lsl r4, r4, #(47-32)
    @str r4, [r10, #0x2c]
    @mov r4, #1
    @lsl r4, r4, #(27)
    @str r4, [r10, #0x1c]
#endif

    _dsb(r4)
.gpio_setup.done:
    b .fastreloc_copy
    @b .slowreloc_copy

@ --------------------------------------------------------------------------------------------------
@ SECTION: TAIL
@ --------------------------------------------------------------------------------------------------

@
@ DATA POOL
@

loc.gpio_base: .word 0x20200000

@
@ END OF STUB
@

__symbol_relocation_stub_end:
    nop
