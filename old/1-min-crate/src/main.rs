#![no_std]
#![no_main]

use core::{
    arch::{asm, global_asm, naked_asm},
    panic::PanicInfo,
};
use core::ptr::with_exposed_provenance_mut;
use proc_bitfield::Bit;
use rp235x_pac::Peripherals;

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

fn xosc_init(peri: &Peripherals) {
    const STARTUP_DELAY : u16 = (((12_000 + 128) / 256) * 64);
    unsafe {
        peri.XOSC.ctrl().write_with_zero(|w| w.freq_range()._1_15mhz());
        peri.XOSC.startup().write_with_zero(|w| w.delay().bits(STARTUP_DELAY));
        peri.XOSC.ctrl().modify(|_, w| w.enable().enable());
        while peri.XOSC.status().read().stable().bit_is_clear() {}
    }
}

fn setup_pll_sys(peri: &Peripherals) {
    // PLL_SYS_150MHZ, XOSC_12MHZ
    // reset system and reference clocks
    peri.CLOCKS.clk_sys_ctrl().modify(|_, w| w.src().clk_ref());
    peri.CLOCKS.clk_ref_ctrl().modify(|_, w| w.src().rosc_clksrc_ph());
    // ... ?

    // initialize PLL_SYS
    peri.RESETS.reset().modify(|_, w| w.pll_sys().clear_bit());
    while peri.RESETS.reset_done().read().pll_sys().bit_is_clear() {}
    peri.PLL_SYS.pwr().reset();
    peri.PLL_SYS.fbdiv_int().reset();
    let refdiv = 3; // 1
    let fbdiv = 150; // 1
    unsafe {
        peri.PLL_SYS.cs().write(|w| w.refdiv().bits(refdiv));
        peri.PLL_SYS.fbdiv_int().write(|w| w.fbdiv_int().bits(fbdiv));
    }
    peri.PLL_SYS.pwr().modify(|_, w| w.pd().clear_bit().vcopd().clear_bit());
    // lock PLL
    let post_div1 = 2; // 5
    let post_div2 = 2; // 2
    peri.PLL_SYS.prim().write(|w| unsafe {
        w.postdiv1().bits(post_div1)
            .postdiv2().bits(post_div2)
    });
    peri.PLL_SYS.pwr().modify(|_, w| w.postdivpd().clear_bit());
}

fn init_downstream_clocks(peri: &Peripherals) {
    // configure
    //  reference clock at CLK_REF = XOSC (12MHz) / 1
    //  system clock at CLK_SYS = PLL_SYS (150MHz) / 1
    //  peripheral clock at CLK_PERI = CLK_SYS

    // CLK_SYS
    // clock_configure_undivided(CLKSRC_CLK_SYS_AUX)
    // clock_configure_internal
    // src      = CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX
    // auxsrc   = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS
    // actual_freq = SYS_CLK_HZ
    // div     = 1 << CLOCKS_CLK_GPOUT0_DIV_INT_LSB
    // unsafe {
    //     peri.CLOCKS.clk_sys_div().write_with_zero(|w| w.int().bits(1));
    //     // clk_sys is glitchless
    //     peri.CLOCKS.clk_sys_ctrl().modify(|_, w| w.src().clk_ref());
    //     while (peri.CLOCKS.clk_sys_selected().read().clk_sys_selected().bits() & 1) == 0 {}
    //
    //     peri.CLOCKS.clk_sys_ctrl().modify(|_, w| w.auxsrc().clksrc_pll_sys());
    //     peri.CLOCKS.clk_sys_ctrl().modify(|_, w| w.src().clksrc_clk_sys_aux());
    //     while (peri.CLOCKS.clk_sys_selected().read().clk_sys_selected().bits() & 2) == 0 {}
    //
    //     peri.CLOCKS.clk_sys_div().write_with_zero(|w| w.int().bits(1));
    // }

    // clk_ref
    // src = xosc
    // aux = 0
    unsafe {
        peri.CLOCKS.clk_ref_div().write_with_zero(|w| w.int().bits(1));
        // do nothing I think?

        // peri.CLOCKS.clk_ref_ctrl().modify(|_, w| w.auxsrc().clksrc_gpin0());
        peri.CLOCKS.clk_ref_ctrl().modify(|_, w| w.src().xosc_clksrc());
        while (peri.CLOCKS.clk_ref_selected().read().clk_ref_selected().bits() & 4) == 0 {}

        peri.CLOCKS.clk_ref_div().write_with_zero(|w| w.int().bits(1));
    }

    // clk_peri (not glitchless) from clk_sys
    // src=0
    // aux=CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS
    unsafe {
        peri.CLOCKS.clk_peri_ctrl().write_with_zero(|w| w.enable().clear_bit());
        // let delay_cycles = 150_000_000 / 150_000_000 + 1;
        // for _ in 0..delay_cycles * 3 {
        //     asm!("nop")
        // }
        peri.CLOCKS.clk_peri_ctrl().modify(|_, w| w.auxsrc().clksrc_pll_sys());
        peri.CLOCKS.clk_peri_ctrl().modify(|_, w| w.enable().set_bit());
        peri.CLOCKS.clk_peri_div().write_with_zero(|w| w.int().bits(2));
        while peri.CLOCKS.clk_peri_ctrl().read().enabled().bit_is_clear() {}
    }
}

pub extern "C" fn kernel_main() -> ! {
    let peri = unsafe { Peripherals::steal() };
    unsafe {
        const B_2225 : u32 = (1 << 25) | (1 << 22);
        // UART0 on GP0,GP1
        peri.IO_BANK0.gpio(0).gpio_ctrl().write_with_zero(|w| w.funcsel().uart());
        peri.IO_BANK0.gpio(1).gpio_ctrl().write_with_zero(|w| w.funcsel().uart());
        peri.PADS_BANK0.gpio(0).write_with_zero(|w| w.drive()._12m_a());
        peri.PADS_BANK0.gpio(1).write_with_zero(|w| w.drive()._12m_a());
        // onboard LED
        peri.IO_BANK0.gpio(25).gpio_ctrl().write_with_zero(|w| w.funcsel().sio());
        peri.PADS_BANK0.gpio(25).write_with_zero(|w| w.drive()._12m_a().pde().set_bit());
        peri.IO_BANK0.gpio(22).gpio_ctrl().write_with_zero(|w| w.funcsel().sio());
        peri.PADS_BANK0.gpio(22).write_with_zero(|w| w.drive()._12m_a().pde().set_bit());
        peri.SIO.gpio_oe().write_with_zero(|w| unsafe { w.bits(B_2225) });

        xosc_init(&peri); // init xosc
        setup_pll_sys(&peri); // init pll_sys from xosc
        init_downstream_clocks(&peri); // init clk_ref from pll_sys

        unsafe {
            // peri.TICKS.tickproc0().ctrl().write_with_zero(|w| w.enable().set_bit()); // arm systick proc0
            // peri.TICKS.tickproc1().ctrl().write_with_zero(|w| w.enable().set_bit()); // arm systick proc1
            // peri.TICKS.ticktimer0().ctrl().write_with_zero(|w| w.enable().set_bit()); // timer0
            // peri.TICKS.ticktimer1().ctrl().write_with_zero(|w| w.enable().set_bit()); // timer1
            peri.TICKS.tickriscv().cycles().write_with_zero(|w| w.bits(12));
            peri.TICKS.tickriscv().ctrl().write_with_zero(|w| w.enable().set_bit()); // riscv
            while peri.TICKS.tickriscv().ctrl().read().running().bit_is_clear() {}
            peri.SIO.mtime_ctrl().write_with_zero(|w| w.en().set_bit());
        }

        // peri.SIO.gpio_out_set().write_with_zero(|w| w.bits(1 << 22));

        // asm!("csrwi 0x320, 0"); // enable cycle counters

        // deassert UART reset
        peri.RESETS.reset().write_with_zero(|w| w.uart0().clear_bit());
        while peri.RESETS.reset_done().read().uart0().bit_is_clear() {}

        // clock = 150MHz
        let uart_clock : u32 = 150_000_000 / 2;
        let baud_rate_div = (8 * uart_clock / 115200) + 1;
        let baud_ibrd = baud_rate_div >> 7;
        let (baud_ibrd, baud_fbrd) = if baud_ibrd == 0 {
            (1, 0)
        } else if baud_ibrd >= 65535 {
            (65535, 0)
        } else {
            (baud_ibrd as u16, ((baud_rate_div & 0x7f) >> 1) as u8)
        };
        peri.UART0.uartibrd().write_with_zero(|w| w.baud_divint().bits(baud_ibrd));
        peri.UART0.uartfbrd().write_with_zero(|w| w.baud_divfrac().bits(baud_fbrd));
        peri.UART0.uartlcr_h().write_with_zero(|w| w.fen().set_bit().wlen().bits(3));
        peri.UART0.uartcr().write_with_zero(|w| w.uarten().set_bit().txe().set_bit().rxe().set_bit());
        peri.UART0.uartdmacr().write_with_zero(|w| w.rxdmae().set_bit().txdmae().set_bit());

        loop {
            for b in "----####".as_bytes() {
                while peri.UART0.uartfr().read().txff().bit_is_set() {}
                peri.UART0.uartdr().write_with_zero(|w| w.data().bits(*b));
            }
        }
    }

    kernel_restart();
}

// pub extern "C" fn kernel_main() -> ! {
//     let io_bank0_base: *mut u32 = with_exposed_provenance_mut(0x4002_8000);
//     unsafe { io_bank0_base.byte_offset(0x0cc).write_volatile(0x5) };
//     let pads_bank0_base: *mut u32 = with_exposed_provenance_mut(0x4003_8000);
//     unsafe { pads_bank0_base.byte_offset(0x068).write_volatile(0x34) };
//     let sio_base: *mut u32 = with_exposed_provenance_mut(0xd000_0000);
//     unsafe { sio_base.byte_offset(0x030).write_volatile(1 << 25) };
//
//     loop {
//         for _ in 0..1000000 {
//             unsafe { asm!("nop") }
//         }
//         unsafe { sio_base.byte_offset(0x028).write_volatile(1 << 25) };
//     }
// }

fn kernel_restart() -> ! {
    #![allow(clippy::empty_loop)]
    loop {}
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    kernel_restart()
}
