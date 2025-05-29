use core::arch::asm;

pub fn __read_tpidrurw() -> u32 {
    let mut out: u32;
    unsafe { asm!("mrc p15, 0, {t}, c13, c0, 2", t = out(reg) out) }
    out
}
pub unsafe fn __write_tpidrurw(v: u32) {
    unsafe { asm!("mcr p15, 0, {t}, c13, c0, 2", t = in(reg) v) }
}

pub fn __read_tpidruro() -> u32 {
    let mut out: u32;
    unsafe { asm!("mrc p15, 0, {t}, c13, c0, 3", t = out(reg) out) }
    out
}
pub unsafe fn __write_tpidruro(v: u32) {
    unsafe { asm!("mcr p15, 0, {t}, c13, c0, 3", t = in(reg) v) }
}

pub fn __read_tpidrprw() -> u32 {
    let mut out: u32;
    unsafe { asm!("mrc p15, 0, {t}, c13, c0, 4", t = out(reg) out) }
    out
}
pub unsafe fn __write_tpidrprw(v: u32) {
    unsafe { asm!("mcr p15, 0, {t}, c13, c0, 4", t = in(reg) v) }
}
