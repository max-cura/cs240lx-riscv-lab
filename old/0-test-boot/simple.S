.section .rp2350.boot_jump
.globl _start0
.extern _start1
_start0:
    j _start1

.section .rp2350.boot_header
.word  0xffffded3
.byte 0x42
.byte 0x01
#define IMG_RP2350  0x1000
#define IMG_RISCV   0x0100
#define IMG_EXE     0x0001
.hword (IMG_RP2350 | IMG_RISCV | IMG_EXE)
.byte 0xff
.hword 0x0001
.byte 0x00
.word 0x00000000
.word 0xab123579

.section .text
.globl _start1
_start1:
    csrw mie, zero
    li t0, 0x40028000 # IO_BANK0_BASE
    li t1, 0x0005 # SIO_25
    sw t1, 0x0cc(t0) # IO_BANK0: GPIO25

    li s0, 0x40038000 # PADS_BANK0_BASE
    li s1, 0x34
    sw s1, 0x068(s0) # PADS_BANK0: GPIO25

    li t2, 0xd0000000 # SIO_BASE
    li t3, (1 << 25)
    sw t3, 0x030(t2)
blinkloop_init:
    li t4, 10000000
blinkloop:
    addi t4, t4, -1
    bnez t4, blinkloop
    sw t3, 0x028(t2)
    j blinkloop_init

loop:
    j loop


