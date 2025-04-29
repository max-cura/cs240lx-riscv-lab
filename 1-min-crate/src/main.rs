#![no_std]
#![no_main]

use core::{
    arch::{asm, global_asm, naked_asm},
    panic::PanicInfo,
};

mod bootrom;

// For LLVM codegen reasons.
global_asm!(
    r#"
.attribute arch, "rv32imac"
"#
);

// Trampoline into kernel entry; note the `.rp2350.boot_jump` - this is to ensure that
// .rp2350.boot_header is in the first 4K of flash.
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

/// Handle low-level processor setup and setup that can't be done inside the Rust abstract machine
/// (e.g. BSS).
#[unsafe(naked)]
#[unsafe(no_mangle)]
pub extern "C" fn kernel_entry() -> ! {
    unsafe extern "C" {
        static __stack_top__: [u32; 0];
        static __bss_start__: [u32; 0];
        static __bss_end__: [u32; 0];
    }
    // NOTE: R-A thinks that naked_asm! requires unsafe (it does not), but until the lint gets fixed
    // we insert an extraneous `unsafe` block around it
    #[allow(unused_unsafe)]
    unsafe {
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
    #![allow(clippy::empty_loop)]
    loop {}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    kernel_restart()
}
