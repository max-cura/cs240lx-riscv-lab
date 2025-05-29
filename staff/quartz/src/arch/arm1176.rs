pub mod coprocessor;
pub mod cpuid;
pub mod cycle;
pub mod mmu;
pub mod pmm;
pub mod sync;
pub mod tpid;

use crate::define_coprocessor_registers;
use core::arch::asm;

define_coprocessor_registers! {
    translation_table_base_0 => p15 0 c2 c0 0;
    translation_table_base_1 => p15 0 c2 c0 1;
    translation_table_base_control => p15 0 c2 c0 2;

    domain_access_control => p15 0 c3 c0 0;

    [safe write] wfi => p15 0 c7 c0 4;
    [safe write] faulty_invalidate_entire_icache => p15 0 c7 c5 0;
    [safe write] flush_prefetch_buffer => p15 0 c7 c5 4;
    [safe write] flush_entire_btac => p15 0 c7 c5 6;

    [safe write] invalidate_entire_dcache => p15 0 c7 c6 0;

    [safe write] invalidate_both_caches => p15 0 c7 c7 0;
    [safe write] clean_entire_dcache => p15 0 c7 c10 0;
    [safe write] dsb => p15 0 c7 c10 4;
    [safe write] dmb => p15 0 c7 c10 5;

    [safe write] clean_and_invalidate_entire_dcache => p15 0 c7 c14 0;
}

#[inline]
pub fn dmb() {
    dmb::write_raw(0);
}

#[inline]
pub fn dsb() {
    dsb::write_raw(0);
}

#[inline]
pub fn prefetch_flush() {
    flush_prefetch_buffer::write_raw(0);
}

#[inline]
pub fn wfi() {
    wfi::write_raw(0);
}

#[inline]
pub fn wfe() {
    unsafe { asm!("wfe") }
}

#[inline]
pub fn sev() {
    unsafe { asm!("sev") }
}

pub const PAGE_SIZE: usize = 0x4000;
