#![no_std]
#![no_main]

use core::{arch::{asm, global_asm, naked_asm}, panic::PanicInfo};

global_asm!(r#"
.attribute arch, "rv32imac"
"#);

global_asm!(r#"
.section .rp2350.boot_jump
.globl _start
_start:
.option push
.option norelax
    la gp, __global_pointer$
.option pop
    j {kernel_entry}
"#, kernel_entry = sym kernel_entry);

#[derive(Debug, Copy, Clone)]
#[repr(packed)]
#[allow(dead_code)]
struct ImageDef {
    typ: u8,
    size: u8,
    flags: u16,
}
#[derive(Debug, Copy, Clone)]
#[repr(packed)]
#[allow(dead_code)]
struct LastItem {
    typ: u8,
    siz: u16,
    pad: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
union BlockItem {
    image_def: ImageDef,
    last_item: LastItem,
}
const _: () = const { assert!(size_of::<BlockItem>() == 4) };

#[derive(Copy, Clone)]
#[allow(dead_code)]
#[repr(C)]
struct Block<const N: usize> {
    header: u32,
    items: [BlockItem; N],
    next: u32,
    footer: u32,
}
#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".rp2350.boot_header")]
static BOOTROM_BLOCK_LOOP : Block<2> = Block {
    header: 0xffff_ded3,
    items: [
        BlockItem { image_def: ImageDef {
            typ: 0x42,
            size: 1,
            flags: 0x1101,
        } },
        BlockItem { last_item: LastItem {
            typ: 0xff,
            siz: 0x0001,
            pad: 0x00,
        } },
    ],
    next: 0,
    footer: 0xab123579
};

#[unsafe(naked)]
#[unsafe(no_mangle)]
pub extern "C" fn kernel_entry() -> ! {
    unsafe extern "C" {
        static __stack_top__: [u32; 0];
        static __bss_start__: [u32; 0];
        static __bss_end__: [u32; 0];
    }
    naked_asm!(
        // disable interrupts
        "csrw mie, zero
        csrci mstatus, 8",

        // clear BSS
        "la t0, {bss_start}
        la t1, {bss_end}
        1:
        bgeu t0, t1, 2f
        sw zero, 0(t0)
        addi t0, t0, 8
        j 1b
        2:",

        // initialize stack and frame pointer
        "la sp, {stack_top}
        andi sp, sp, -16
        add fp, sp, zero",

        // jump to main function
        "j {kernel_main}",

        bss_start = sym __bss_start__,
        bss_end = sym __bss_end__,
        stack_top = sym __stack_top__,
        kernel_main = sym kernel_main,
    )
}

pub extern "C" fn kernel_main() -> ! {
    let io_bank0_base: *mut u32 = core::ptr::with_exposed_provenance_mut(0x4002_8000);
    unsafe { io_bank0_base.byte_offset(0x0cc).write_volatile(0x5) };
    let pads_bank0_base: *mut u32 = core::ptr::with_exposed_provenance_mut(0x4003_8000);
    unsafe { pads_bank0_base.byte_offset(0x068).write_volatile(0x34) };
    let sio_base: *mut u32 = core::ptr::with_exposed_provenance_mut(0xd000_0000);
    unsafe { sio_base.byte_offset(0x030).write_volatile(1 << 25) };

    loop {
        for _ in 0..1000000 {
            unsafe { asm!("nop") }
        }
        unsafe { sio_base.byte_offset(0x028).write_volatile(1 << 25) };
    }
}

fn kernel_restart() -> ! {
    loop {}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    kernel_restart()
}