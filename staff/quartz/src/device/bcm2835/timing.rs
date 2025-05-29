use crate::arch::arm1176::dsb;
use bcm2835_lpa::SYSTMR;
use core::intrinsics::unlikely;
use core::time::Duration;

// we want a version without __dsb()'s for use in tight loops where we have no other peripherals
// (e.g. the delay functions).
unsafe fn __floating_time_unguarded(st: &SYSTMR) -> u64 {
    let hi32 = { st.chi().read().bits() as u64 } << 32;
    hi32 | { st.clo().read().bits() as u64 }
}

/// Read a time in microseconds from the floating system timer. The clock rate is 1MHz.
/// When possible, you should prefer the use of [`Instant`].
pub fn __floating_time(st: &SYSTMR) -> u64 {
    // CHI|CLO runs on a 1MHz oscillator
    dsb();
    let t = unsafe { __floating_time_unguarded(st) };
    dsb();
    t
}

impl crate::device::timing::FloatingTimer for SYSTMR {
    //
}
