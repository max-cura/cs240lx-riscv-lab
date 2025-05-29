#![allow(internal_features)]
#![feature(core_intrinsics)]
#![feature(array_ptr_get)]
#![feature(pointer_is_aligned_to)]
#![feature(vec_into_raw_parts)]
#![feature(new_range_api)]
#![feature(range_into_bounds)]
#![feature(get_disjoint_mut_helpers)]
// don't import the standard library
#![no_std]
// we'll manage program entry on our own, thank you very much
#![no_main]

extern crate alloc;

#[global_allocator]
// static HEAP: embedded_alloc::TlsfHeap = embedded_alloc::TlsfHeap::empty();
static HEAP: linked_list_allocator::LockedHeap = linked_list_allocator::LockedHeap::empty();

use core::arch::{asm, global_asm, naked_asm};
use core::panic::PanicInfo;
use critical_section::RawRestoreState;
use okboot_common::INITIAL_BAUD_RATE;
//use quartz::arch::arm1176::mmu::{MMUEnabledFeaturesConfig, __set_mmu_enabled_features};
use rp235x_pac::Peripherals;

mod bootrom;
mod buf;
pub mod legacy;
mod protocol;
mod stub;
pub mod timeouts;

#[unsafe(no_mangle)]
pub extern "C" fn __aeabi_unwind_cpp_pr0() {}

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
        static __flash_end__: [u32; 0];
        static __data_start__: [u32; 0];
        static __data_end__: [u32; 0];
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

            // load data segment
            "la t0, {data_start}
            la t1, {data_end}
            la t2, {flash_end}
            3:
            bgeu t0, t1, 4f
            lw t3, 0(t2)
            sw t3, 0(t0)
            addi t0, t0, 4
            addi t2, t2, 4
            j 3b
            4:
            ",

            // initialize stack and frame pointer
            "la sp, {stack_top}
            andi sp, sp, -16
            add fp, sp, zero",

            // jump to main function
            "j {kernel_main}",

            bss_start = sym __bss_start__,
            bss_end = sym __bss_end__,
            stack_top = sym __stack_top__,
            flash_end = sym __flash_end__,
            data_start = sym __data_start__,
            data_end = sym __data_end__,
            kernel_main = sym kernel_main,
        )
    }
}

pub fn led_set(on: bool) {
    let peripherals = unsafe { rp235x_pac::Peripherals::steal() };
    let on = if on { 1 } else { 0 };
    let bit = on << 25;
    unsafe {
        peripherals
            .SIO
            .gpio_out()
            .modify(|r, w| w.bits((r.bits() & !(1 << 25)) | bit))
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn kernel_main() -> ! {
    // NOTE: It seems to be impractical/impossible to zero out the BSS in life-after-main, so we
    //       now do it in life-before-main (specifically, in _start in boot.S).
    // This is mostly because it is UB for the BSS to be uninitialized during AM execution, and also
    // because there is no way to get a pointer with provenance for the whole BSS section.

    let peripherals = unsafe { rp235x_pac::Peripherals::steal() };

    unsafe {
        // UART0 on GP0,GP1
        peripherals
            .IO_BANK0
            .gpio(0)
            .gpio_ctrl()
            .write_with_zero(|w| w.funcsel().uart());
        peripherals
            .IO_BANK0
            .gpio(1)
            .gpio_ctrl()
            .write_with_zero(|w| w.funcsel().uart());
        peripherals
            .PADS_BANK0
            .gpio(0)
            .write_with_zero(|w| w.drive()._12m_a());
        peripherals
            .PADS_BANK0
            .gpio(1)
            .write_with_zero(|w| w.drive()._12m_a().od().clear_bit().ie().set_bit());
        // onboard LED
        const LED_BITS: u32 = 1 << 25;
        peripherals
            .IO_BANK0
            .gpio(25)
            .gpio_ctrl()
            .write_with_zero(|w| w.funcsel().sio());
        peripherals
            .PADS_BANK0
            .gpio(25)
            .write_with_zero(|w| w.drive()._12m_a().pde().set_bit());
        peripherals
            .SIO
            .gpio_oe()
            .write_with_zero(|w| w.bits(LED_BITS));
    }

    let initials = [
        peripherals.XOSC.ctrl().read().bits(),
        peripherals.XOSC.status().read().bits(),
        peripherals.XOSC.dormant().read().bits(),
        peripherals.XOSC.startup().read().bits(),
        peripherals.ROSC.ctrl().read().bits(),
        peripherals.PLL_SYS.cs().read().bits(),
        peripherals.PLL_SYS.pwr().read().bits(),
        peripherals.PLL_SYS.fbdiv_int().read().bits(),
        peripherals.PLL_SYS.prim().read().bits(),
        peripherals.PLL_SYS.intr().read().bits(),
        peripherals.PLL_SYS.inte().read().bits(),
        peripherals.PLL_SYS.intf().read().bits(),
        peripherals.PLL_SYS.ints().read().bits(),
        peripherals.PLL_USB.cs().read().bits(),
        peripherals.PLL_USB.pwr().read().bits(),
        peripherals.PLL_USB.fbdiv_int().read().bits(),
        peripherals.PLL_USB.prim().read().bits(),
        peripherals.PLL_USB.intr().read().bits(),
        peripherals.PLL_USB.inte().read().bits(),
        peripherals.PLL_USB.intf().read().bits(),
        peripherals.PLL_USB.ints().read().bits(),
        peripherals.CLOCKS.clk_peri_ctrl().read().bits(),
        peripherals.CLOCKS.clk_peri_div().read().bits(),
        peripherals.CLOCKS.clk_peri_selected().read().bits(),
        peripherals.CLOCKS.clk_ref_ctrl().read().bits(),
        peripherals.CLOCKS.clk_ref_div().read().bits(),
        peripherals.CLOCKS.clk_ref_selected().read().bits(),
        peripherals.RESETS.reset().read().bits(),
        peripherals.RESETS.reset_done().read().bits(),
    ];

    unsafe {
        // Initialize XOSC crystal oscillator; it's a 12MHz clock on the Pico 2 (we know this
        // because the bootrom can only use the USB mass storage device if the XOSC runs at 12MHz)
        quartz::arch::hazard3::clocks::init_xosc(&peripherals.XOSC);
        // Initialize the PLL_SYS clock at 150MHz, using the XOSC as the basis
        quartz::arch::hazard3::clocks::init_pll_sys_from_xosc(
            quartz::arch::hazard3::clocks::PLLConfig::_150mhz_from_xosc(),
            &peripherals.PLL_SYS,
            &peripherals.RESETS,
            &peripherals.CLOCKS,
        );
        // Initialize CLK_PERI from PLL_SYS at 150MHz; CLK_PERI is used by the UARTs and SPI.
        quartz::arch::hazard3::clocks::init_clk_peri_from_pll_sys(1, &peripherals.CLOCKS);
        // Initialize CLK_REF from the XOSC at 12MHz; this is used by the various timers and tick
        // generators
        quartz::arch::hazard3::clocks::init_clk_ref_from_xosc(1, &peripherals.CLOCKS);
        // Initialize the RISC-V timer (e.g. mtime/mtimeh)
        quartz::arch::hazard3::clocks::init_riscv_timer_from_clk_ref(
            12,
            &peripherals.TICKS,
            &peripherals.SIO,
        );
    };

    // Initialize UART at initial baud rate
    let uart = quartz::device::rp2350::PL011Uart::init(
        INITIAL_BAUD_RATE,
        150_000_000,
        &peripherals.UART0,
        &peripherals.RESETS,
    );

    // unsafe {
    //     core::ptr::with_exposed_provenance_mut::<u32>(0x2005_0000).write_volatile(0u32);
    // }

    // Create a floating timer object
    let ft = quartz::device::rp2350::RiscvTimer::new();

    // Delay 100ms for superstition
    // XXX: it's probably fine to ignore this here
    //delay_millis(ft, 100);

    // TODO: enable icache/dcache/branch prediction

    // Enable heap
    unsafe {
        unsafe extern "C" {
            static __heap_start__: [u8; 0];
            static __heap_end__: [u8; 0];
        }
        let heap_start = &raw const __heap_start__;
        let heap_end = &raw const __heap_end__;
        let heap_len = heap_end.byte_offset_from_unsigned(heap_start);

        asm!("csrci mstatus, 0");

        HEAP.lock()
            .init(heap_start.cast::<u8>().cast_mut(), heap_len);

        asm!("csrci mstatus, 0");

        led_set(true);

        legacy_print_string_blocking!(
            &uart,
            "initialized HEAP at {:08x} with length {}B\r\n",
            heap_start.addr(),
            heap_len
        );
    }

    for (i, val) in initials.into_iter().enumerate() {
        legacy_print_string_blocking!(&uart, "{i} = {val:08x}\r\n");
    }

    // legacy_print_string_blocking!(&uart, "testing timer");
    // delay_millis(&ft, 10_000);
    // legacy_print_string_blocking!(&uart, "done (should have been 10s)");

    // legacy_print_string_blocking!(&uart, "initializing MMU\n");

    unsafe extern "C" {
        static __flash_end__: [u32; 0];
    }
    legacy_print_string_blocking!(
        &uart,
        "__flash_end__ = {:08x}",
        (&raw const __flash_end__).addr()
    );

    protocol::run(&uart, &ft);

    legacy_print_string_blocking!(&uart, "protocol failure; restarting\n");

    __symbol_kreboot()
}

#[unsafe(no_mangle)]
pub extern "C" fn __symbol_kreboot() -> ! {
    // TODO: watchdog timer
    loop {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // TODO: refactor

    let peripherals = unsafe { Peripherals::steal() };
    let uart = quartz::device::rp2350::PL011Uart::init(
        INITIAL_BAUD_RATE,
        150_000_000,
        &peripherals.UART0,
        &peripherals.RESETS,
    );
    //mini_uart::muart1_init(&peri.GPIO, &peri.AUX, &peri.UART1, 270);

    if let Some(loc) = info.location() {
        legacy_print_string_blocking!(
            &uart,
            "[device]: Panic occurred at file '{}' line {}:\n",
            loc.file(),
            loc.line()
        );
    } else {
        legacy_print_string_blocking!(&uart, "[device]: Panic occurred at [unknown location]\n");
    }
    let msg = info.message();
    use core::fmt::Write as _;
    // let bub = unsafe { &mut *BOOT_UMSG_BUF.0.get() };
    // bub.clear();
    let mut s = alloc::string::String::new();
    if core::fmt::write(&mut s, format_args!("{}\n", msg)).is_err() {
        legacy_print_string_blocking!(
            &uart,
            "[device]: [failed to write message to format buffer]\n"
        );
    }
    if legacy::fmt::UartWrite::new(&uart).write_str(&s).is_err() {
        legacy_print_string_blocking!(&uart, "[device]: [failed to write message to uart]\n");
    }
    // } else {
    //     legacy_print_string_blocking!(&peri.UART1, "[device]: [no message]");
    // }
    legacy_print_string_blocking!(&uart, "[device]: rebooting.\n");

    __symbol_kreboot()
}

// struct MyCriticalSection;
// critical_section::set_impl!(MyCriticalSection);

// unsafe impl critical_section::Impl for MyCriticalSection {
//     unsafe fn acquire() -> RawRestoreState {
//         unsafe {
//             asm!("csrw mie, zero");
//         }
//         0
//     }

//     unsafe fn release(_token: RawRestoreState) {
//         // TODO
//         unsafe { asm!("nop") }
//     }
// }

use critical_section::*;
struct SingleHartCriticalSection;
set_impl!(SingleHartCriticalSection);

unsafe impl Impl for SingleHartCriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        let mut mstatus: usize;
        unsafe { core::arch::asm!("csrrci {}, mstatus, 0b1000", out(reg) mstatus) };
        let was_active = (mstatus & 0x8) != 0;
        was_active
    }

    unsafe fn release(was_active: RawRestoreState) {
        // Only re-enable interrupts if they were enabled before the critical section.
        if was_active {
            unsafe { core::arch::asm!("csrsi mstatus, 0b1000") };
        }
    }
}
