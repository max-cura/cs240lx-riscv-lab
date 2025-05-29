use crate::arch::arm1176::{dmb, sev, wfe};
use core::arch::asm;
use lock_api::{GuardSend, RawMutex, RawMutexFair};

const NEXT_LSB: u32 = 1 << 16;

#[repr(C)]
pub union RawTicketLock {
    raw: u32,
    ordered: TicketLockInner,
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
struct TicketLockInner {
    owner: u16,
    next: u16,
}

unsafe impl RawMutex for RawTicketLock {
    const INIT: Self = Self { raw: 0 };
    type GuardMarker = GuardSend;

    fn lock(&self) {
        let mut raw_val: u32;
        let mut ticket: u32;
        unsafe {
            /* linux has prefetchw here, but it doesn't seem like that's implemented on arm1176jzf-s */
            /* atm, locality is arbitrarily set as 1 */
            core::intrinsics::prefetch_write_data(core::ptr::from_ref(&self.raw), 1);
            asm!(
            "2:",
            "ldrex {t}, [{la}]",
            "add {r}, {t}, {incr}",
            "strex {t2}, {r}, [{la}]",
            "teq {t2}, #0",
            "bne 2b",
            t = out(reg) raw_val,
            la = in(reg) &self.raw,
            r = out(reg) ticket,
            t2 = out(reg) _,
            incr = in(reg) NEXT_LSB,
            );
        }
        let mut raw_val = Self { raw: raw_val };
        unsafe {
            while raw_val.ordered.owner != raw_val.ordered.next {
                wfe();
                raw_val.ordered.owner =
                    core::intrinsics::atomic_load_acquire(core::ptr::from_ref(&self.ordered.owner));
            }
        }
        let _ = ticket;

        /* linux has smb_mb here, but it doesn't seem like that's implemented on arm1176jzf-s */
        dmb();
    }

    fn try_lock(&self) -> bool {
        let mut contended: u32;
        let mut atomic: u32;
        loop {
            unsafe {
                asm!(
                "ldrex {t}, [{la}]",
                "mov {s}, #0",
                "subs {r}, {t}, {t}, ror #16",
                "addeq {t}, {t}, {incr}",
                "strexeq {s}, {t}, [{la}]",
                t = out(reg) _,
                s = out(reg) atomic,
                r = out(reg) contended,
                la = in(reg) &self.raw,
                incr = in(reg) NEXT_LSB,
                );
            }
            if atomic == 0 {
                break;
            }
        }
        if contended == 0 {
            /* smp_mb */
            dmb();
            true
        } else {
            false
        }
    }

    unsafe fn unlock(&self) {
        /* smp_mb */
        dmb();
        unsafe {
            core::intrinsics::atomic_xadd_acqrel(core::ptr::from_ref(&self.raw).cast_mut(), 1);
        }
        sev();
    }
}

unsafe impl RawMutexFair for RawTicketLock {
    unsafe fn unlock_fair(&self) {
        unsafe { self.unlock() }
    }
}

pub type TicketLock<T> = lock_api::Mutex<RawTicketLock, T>;
pub type TicketLockGuard<'a, T> = lock_api::MutexGuard<'a, RawTicketLock, T>;
