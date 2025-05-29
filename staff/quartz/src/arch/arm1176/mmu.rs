use crate::arch::arm1176::dsb;
use core::arch::asm;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct MMUEnabledFeaturesConfig {
    pub dcache: Option<bool>,
    pub icache: Option<bool>,
    pub brpdx: Option<bool>,
}

pub unsafe fn __set_mmu_enabled_features(config: MMUEnabledFeaturesConfig) {
    let mut cr_on = 0;
    let mut cr_off = 0;
    const CR_DCACHE_BIT: u32 = 1 << 2;
    const CR_BRPDX_BIT: u32 = 1 << 11;
    const CR_ICACHE_BIT: u32 = 1 << 12;
    if let Some(dcache) = config.dcache {
        if dcache {
            cr_on |= CR_DCACHE_BIT
        } else {
            cr_off |= CR_DCACHE_BIT
        }
    }
    if let Some(icache) = config.icache {
        if icache {
            cr_on |= CR_ICACHE_BIT
        } else {
            cr_off |= CR_ICACHE_BIT
        }
    }
    if let Some(brpdx) = config.brpdx {
        if brpdx {
            cr_on |= CR_BRPDX_BIT
        } else {
            cr_off |= CR_BRPDX_BIT
        }
    }
    if cr_on != 0 || cr_off != 0 {
        unsafe {
            asm!(
            "mrc p15, 0, {t}, c1, c0, 0",
            "orr {t}, {t}, {cr_on}",
            "and {t}, {t}, {cr_off}",
            "mcr p15, 0, {t}, c1, c0, 0",
            t = out(reg) _,
            cr_on = in(reg) cr_on,
            cr_off = in(reg) !cr_off,
            );
        }
    }
}

#[inline(never)]
pub unsafe fn __disable_mmu() {
    // Clear bit 2 to 0 in the CP15 Control Register c1 of the corresponding world, to disable the
    // Data Cache. You must disable the Data Cache in the corresponding world before, or at the
    // same time as, disabling the MMU.
    // Note
    // If the MMU is enabled, then disabled, and subsequently re-enabled in the same world, the
    // contents of the TLBs for this world are preserved. If these are now invalid, you must
    // invalidate the TLBs in the corresponding world before you re-enable the MMU, see c8,
    // TLB Operations Register on page 3-86.
    // 2. Clear bit 0 to 0 in the CP15 Control Register c1 of the corresponding world.
    dsb();
    // disable dcache
    unsafe {
        asm!("mrc p15, 0, {t}, c1, c0, 0", "and {t}, {t}, {off}", "mcr p15, 0, {t}, c1, c0, 0",
        t = out(reg) _,
        off = in(reg) !4,
        );
    }
    unsafe {
        asm!("mrc p15, 0, {t}, c1, c0, 0", "and {t}, {t}, {off}", "mcr p15, 0, {t}, c1, c0, 0",
        t = out(reg) _,
        off = in(reg) !1,
        );
    }
    dsb();
}

#[inline(never)]
pub unsafe fn __init_mmu(ttb_ptr: *mut u32) {
    // Initialize TTB
    unsafe {
        // TTB
        __init_mmu_translation_table(ttb_ptr);
        // DAC, PRRR, NMRR
        __init_mmu_tex_remap();
    }

    // Mainline MMU initialization

    // translation table base register 0
    let ttb0 = (ttb_ptr as usize as u32) | 0b01001;
    unsafe {
        asm!(
        "mcr p15, 0, {t}, c2, c0, 0",
        t = in(reg) ttb0,
        );
    }

    // translation table base control register
    let ttbcr = 0b100000;
    unsafe {
        asm!(
        "mcr p15, 0, {t}, c2, c0, 2",
        t = in(reg) ttbcr,
        );
    }

    // XP=1 (use ARMv6 page tables)
    let cr_on = 1 << 23;
    // I=0 (disable icache)
    let cr_off = 1 << 12;
    unsafe {
        asm!(
        "mrc p15, 0, {t}, c1, c0, 0",
        "orr {t}, {t}, {cr_on}",
        "and {t}, {t}, {cr_off}",
        "mcr p15, 0, {t}, c1, c0, 0",
        t = out(reg) _,
        cr_on = in(reg) cr_on,
        cr_off = in(reg) !cr_off,
        );
    }

    // invalidate entire icache, and flush branch target cache, and globally flush BTAC
    unsafe {
        asm!(
        "mcr p15, 0, {t}, c7, c5, 0",
        t = in(reg) 0,
        );
    }

    // enable MMU
    unsafe {
        asm!(
        "mrc p15, 0, {t}, c1, c0, 0",
        "orr {t}, {t}, #1",
        "mcr p15, 0, {t}, c1, c0, 0",
        t = out(reg) _,
        )
    }
}

pub unsafe fn __init_mmu_translation_table(ttb_ptr: *mut u32) {
    // uart1_sendln_bl!("__init_mmu_translation_table({ttb_ptr:p})");
    // 0000_0000..1f00_0000 is ARM SDRAM
    // 1f00_0000..1fff_ffff is VC SDRAM iff configured to support a mmap'd display (that is not the
    //                         case)
    // 2000_0000..20ff_ffff is peripheral memory
    // end of physical memory is 4000_0000 (1GB)
    // a full TTB is 16KB, so 4K entries, each entry represents 1MB
    // initially, we map:
    for ttei in 0..0x1000 {
        unsafe { ttb_ptr.offset(ttei).write_volatile(0) };
    }
    // uart1_sendln_bl!("ttb_ptr={ttb_ptr:p}");
    fn tt_supersection(index: u32, tex: u32, c: u32, b: u32) -> u32 {
        #[allow(non_snake_case)]
        {
            let ssba = index << 24;
            let NS = 0 << 19; // stay in the secure world
            let nG = 0 << 17; // Global
            let S = 0 << 16; // Non-Shared memory
            let APX = 0 << 15; // privileged mode can RW
            let TEX = tex << 12;
            let AP = 3 << 10; // user mode can RW
            let P = 0 << 9; // no ECC
            let domain = 0x0 << 5; // (4bit) default to D0
            let XN = 0 << 4; // allow W|X, for now
            let C = c << 3;
            let B = b << 2;
            ((1 << 18) | 0x2) | ssba | NS | nG | S | APX | TEX | AP | P | domain | XN | C | B
        }
    }

    //

    //  0000_0000..1000_0000 to 0000_0000..1000_0000 (256)
    for si in 0..512 {
        let ttei = 0x000 + si;
        let ssi = ((si + 0) >> 4) as u32;
        let entry_ptr = unsafe { ttb_ptr.offset(ttei) };
        // BB=11, write back, no allocate on write
        // AA=11, write back, no allocate on write
        // let val = tt_supersection(ssi, 0b111, 1, 1); // <-- with TEX Remap OFF
        // TEMP: uncached
        // restore to: 0b001, 0, 1
        let val = tt_supersection(ssi, 0b001, 0, 0);
        // uart1_sendln_bl!("BLK1 writing {val:08x} to {entry_ptr:p}");
        unsafe { entry_ptr.write_volatile(val) };
    }
    //  2000_0000..2100_0000 to 2000_0000..2100_0000 (16)
    for si in 0..16 {
        let ttei = 0x200 + si;
        let ssi = ((si + 512) >> 4) as u32;
        let entry_ptr = unsafe { ttb_ptr.offset(ttei) };
        let val = tt_supersection(ssi, 0b000, 0, 1);
        // uart1_sendln_bl!("DEV writing {val:08x} to {entry_ptr:p}");
        unsafe { entry_ptr.write_volatile(val) };
    }
}

pub unsafe fn __init_mmu_tex_remap() {
    // not implemented: inner allocate on write (01)
    // inner non-cacheable mandatory                            [supported]
    // inner write-through mandatory                            [supported]
    // inner write-back optional (else inner write-through)     [supported]
    // outer non-cacheable mandatory
    // outer write-through optional (else outer non-cacheable)
    // outer write-back optional (else outer write-through)

    // for normal memory, use 5, so XX101
    // for device memory, use 1, so XX001
    // Device.Shareable = PRRR[16] (1)
    // Normal.Shareable = PRRR[18] (0)
    // PRRR = NSNNNNDS
    // Default:
    //  0   S/00/00
    //  1   D/00/00
    //  2   N/10/10
    //  3   N/11/11
    //  4   N/00/00
    //  5   N/10/01
    //  6   S/00/00
    //  7   N/01/01

    // let mut prrr : u32;
    // let mut nmrr : u32;
    // asm!(
    //     "mrc p15, 0, {prrr}, c10, c2, 0",
    //     "mrc p15, 0, {nmrr}, c10, c2, 1",
    //     prrr = out(reg) prrr,
    //     nmrr = out(reg) nmrr,
    // );
    // uart1_sendln_bl!("prrr={prrr:08x} nmrr={nmrr:08x}");
    //
    // let prrr = 0x0009_8aa4;
    // let nmrr = 0b0100_0100_1110_0000_0100_1000_1110_0000;
    // asm!(
    //     "mcr p15, 0, {prrr}, c10, c2, 0",
    //     "mcr p15, 0, {nmrr}, c10, c2, 1",
    //     prrr = in(reg) prrr,
    //     nmrr = in(reg) nmrr,
    // );

    // DAC
    //  0 -> 11 // manager
    //  1 -> 01 // client
    //  2 -> 00 // fault
    let dac = 0x0000_0007;
    unsafe {
        asm!(
        "mcr p15, 0, {t}, c3, c0, 0",
        t = in(reg) dac
        );
    }

    // CR.TR = 1 (enable TEX Remap)
    unsafe {
        asm!(
        "mrc p15, 0, {t}, c1, c0, 0",
        "orr {t}, {t}, {cr_on}",
        "mcr p15, 0, {t}, c1, c0, 0",
        t = out(reg) _,
        cr_on = in(reg) { 1 << 28 },
        );
    }
}
