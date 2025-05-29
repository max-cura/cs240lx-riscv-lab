//! Clock management on the Hazard3 RISC-V core.

use rp235x_pac::{CLOCKS, PLL_SYS, RESETS, SIO, TICKS, XOSC};

/// Initialize the XOSC; in the Pico 2, this is a 12MHz crystal.
pub unsafe fn init_xosc(xosc: &XOSC) {
    // 12MHz
    const STARTUP_DELAY: u16 = ((12_000 + 128) / 256) * 64;
    unsafe {
        xosc.ctrl().write_with_zero(|w| w.freq_range()._1_15mhz());
        xosc.startup()
            .write_with_zero(|w| w.delay().bits(STARTUP_DELAY));
        xosc.ctrl().modify(|_, w| w.enable().enable());
        // wait until the XOSC is stable
        while xosc.status().read().stable().bit_is_clear() {}
    }
}

/// Formula is `(REF / reference_divider) * feedback_divisor / (post_divisor_1 * post_divisor_2)`.
/// For information about the possible ranges of the fields, see the manual.
pub struct PLLConfig {
    pub reference_divider: u8,
    pub feedback_divisor: u16,
    pub post_divider_1: u8,
    pub post_divider_2: u8,
}
impl PLLConfig {
    /// Will set PLL to 150MHz, if the reference is a 12MHz XOSC.
    pub fn _150mhz_from_xosc() -> Self {
        Self {
            reference_divider: 3,
            feedback_divisor: 150,
            post_divider_1: 2,
            post_divider_2: 2,
        }
    }
}

/// Initialize the PLL_SYS from XOSC. Will reset CLK_SYS and CLK_REF.
pub unsafe fn init_pll_sys_from_xosc(
    config: PLLConfig,
    pll_sys: &PLL_SYS,
    resets: &RESETS,
    clocks: &CLOCKS,
) {
    // -- Reset system and reference clocks --
    clocks.clk_sys_ctrl().modify(|_, w| w.src().clk_ref());
    clocks
        .clk_ref_ctrl()
        .modify(|_, w| w.src().rosc_clksrc_ph());

    // -- Initialize PLL_SYS --
    // 1. De-assert PLL_SYS reset in RESETS block
    resets.reset().modify(|_, w| w.pll_sys().clear_bit());
    while resets.reset_done().read().pll_sys().bit_is_clear() {}
    // 2. Carry out resets in the PLL_SYS block
    pll_sys.pwr().reset(); // restore power mode register to reset conditions
    pll_sys.fbdiv_int().reset(); // restore feedback divisor register to reset conditions
    // 3. Set up VCO
    pll_sys
        .cs()
        .write(|w| unsafe { w.refdiv().bits(config.reference_divider) });
    pll_sys
        .fbdiv_int()
        .write(|w| unsafe { w.fbdiv_int().bits(config.feedback_divisor) });
    // 4. Enable power to PLL & VCO
    pll_sys
        .pwr()
        .modify(|_, w| w.pd().clear_bit().vcopd().clear_bit());
    // 5. Set post dividers
    pll_sys.prim().write(|w| unsafe {
        w.postdiv1()
            .bits(config.post_divider_1)
            .postdiv2()
            .bits(config.post_divider_2)
    });
    // 6. Enable power to post dividers
    pll_sys.pwr().modify(|_, w| w.postdivpd().clear_bit());
}

/// Initialize CLK_REF from the XOSC at 12MHz. CLK_REF is used by the various timer subsystems.
pub unsafe fn init_clk_ref_from_xosc(divider: u8, clocks: &CLOCKS) {
    // 1. Set the reference divider
    // XXX: technically, we only do this if increasing the divider, but we're initializing so I
    // think we just do this unconditionally (?)
    unsafe {
        clocks
            .clk_ref_div()
            .write_with_zero(|w| w.int().bits(divider))
    }
    // 2. Switch the clock source
    clocks.clk_ref_ctrl().modify(|_, w| w.src().xosc_clksrc());
    // clk_ref_selected is a 1-hot status for the clock source; xosc_clksrc() is 2, so:
    let xosc_src_mask = 1 << 2;
    while (clocks.clk_ref_selected().read().clk_ref_selected().bits() & xosc_src_mask) == 0 {}
    // 3. Rewrite the reference divider
    unsafe {
        clocks
            .clk_ref_div()
            .write_with_zero(|w| w.int().bits(divider))
    }
}

/// Initialize CLK_PERI from the PLL_SYS.
pub unsafe fn init_clk_peri_from_pll_sys(divider: u8, clocks: &CLOCKS) {
    // Note that the process is slightly different from CLK_REF since CLK_REF is glitchless but
    // CLK_PERI is not.

    // 1. Disable by setting all bits to zero (the key is that we clear the ENABLE bit)
    unsafe { clocks.clk_peri_ctrl().write_with_zero(|w| w) };

    // 2. Set the source
    clocks
        .clk_peri_ctrl()
        .modify(|_, w| w.auxsrc().clksrc_pll_sys());

    // 3. Enable the clock
    clocks.clk_peri_ctrl().modify(|_, w| w.enable().set_bit());

    // 4. Set the divider
    unsafe {
        clocks
            .clk_peri_div()
            .write_with_zero(|w| w.int().bits(divider))
    };

    // 5. Wait for clock to be enabled
    while clocks.clk_peri_ctrl().read().enabled().bit_is_clear() {}
}

/// Initialize RISC-V timer against CLK_REF. Note that CLK_REF must be initialized already.
/// `divider` is the number of times CLK_REF should fire before the RISC-V timer advances.
///
/// Somewhat oddly, the RISC-V timer involves the SIO block since it can be used from the ARM side,
/// so we need to configure that as well (even if we're using it from the RISC-V side).
pub unsafe fn init_riscv_timer_from_clk_ref(divider: u32, ticks: &TICKS, sio: &SIO) {
    // 1. Set divider
    unsafe {
        ticks
            .tickriscv()
            .cycles()
            .write_with_zero(|w| w.bits(divider))
    }

    // 2. Enable
    unsafe {
        ticks
            .tickriscv()
            .ctrl()
            .write_with_zero(|w| w.enable().set_bit())
    }
    while ticks.tickriscv().ctrl().read().running().bit_is_clear() {}

    // 3. Enable in SIO
    unsafe { sio.mtime_ctrl().write_with_zero(|w| w.en().set_bit()) }
}
