#define IO_BANK0_BASE 0x40028000
#define PADS_BANK0_BASE 0x40038000
#define SIO_BASE 0xd0000000

.section ".text.boot"
.global start
start:
/*    li s0, IO_BANK0_BASE
    li t0, 0x5
    sw t0, 0x0cc(s0)
    li s1, PADS_BANK0_BASE
    li t0, 0x34
    sw t0, 0x068(s1)
*/
    li s2, 0xd0000000
    li t0, 1 << 25
    sw t0, 0x030(s2)
loop:
    li t1, 1000000
delay:
    addi t1, t1, -1
    bnez t1, delay
    sw t0, 0x028(s2)
    j loop
