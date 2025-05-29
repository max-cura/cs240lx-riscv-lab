use core::{hint::unlikely, time::Duration};

/// Trait that describes typical floating timers available on many systems. Expected to be a 1MHz
/// timer whose zero point is unknown.
pub trait FloatingTimer {
    /// Get the time in microseconds since some past instant.
    fn floating_time(&self) -> u64;
    /// A version of [`floating_time`] that must be called consecutively; by default equivalent
    /// to [`floating_time`], but some platforms that need extra synchronization when switching
    /// between peripheral devices may find this a useful optimization.
    unsafe fn floating_time_consecutive(&self) -> u64 {
        self.floating_time()
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Instant {
    floating_micros: u64,
}
impl Instant {
    pub fn now<FT: FloatingTimer>(ft: &FT) -> Self {
        Self {
            floating_micros: ft.floating_time(),
        }
    }
    pub fn elapsed<FT: FloatingTimer>(&self, ft: &FT) -> Duration {
        let current_time = ft.floating_time();
        Duration::from_micros(current_time.wrapping_sub(self.floating_micros))
    }
}

/// Blocking wait for (at least) `milliseconds` milliseconds.
/// Implemented on top of [`delay_micros`]; see that function's documentation for timing guarantees.
pub fn delay_millis<FT: FloatingTimer>(ft: &FT, mut milliseconds: u64) {
    const MAX_MILLIS_PER_STEP: u64 = u64::MAX / 1000;
    const SATURATE_TO_MICROS: u64 = MAX_MILLIS_PER_STEP * 1000;
    while milliseconds > MAX_MILLIS_PER_STEP {
        let microseconds = SATURATE_TO_MICROS;
        delay_micros(ft, microseconds);
        milliseconds -= MAX_MILLIS_PER_STEP;
    }
    let microseconds = milliseconds * 1000;
    delay_micros(ft, microseconds);
}

/// Blocking wait for (at least) `microseconds` microseconds. We can only make a guarantee that it
/// waits for at least `microseconds`, but in practice, in a no-interrupts setting, it should be
/// exact due to the difference in clock rate.
pub fn delay_micros<FT: FloatingTimer>(ft: &FT, microseconds: u64) {
    #[cfg(feature = "bcm2835")]
    ::quartz::arch::arm1176::dsb();

    let start = unsafe { ft.floating_time_consecutive() };
    let end = start.wrapping_add(microseconds);
    if unlikely(end < start) {
        if unlikely(microseconds > (u32::MAX as u64)) {
            // wraparound: end < start <= u64::MAX
            const U64_HALF: u64 = u64::MAX / 2;
            // The first instinct is to just write:
            //  while now >= start || now < end {}
            // however, this breaks down for end == start-1 for micros=u64::MAX
            // so, we check whether we passed 0, and we can therefore use:
            //  while !passed_zero || now < end {}
            // however, this still breaks down for start=u64::MAX, micros=u64::MAX
            // so, we also check whether we passed u64::MAX/2 after passing zero; and once we pass
            // u64::MAX/2, if we go below u64::MAX/2 again, immediately stop the loop.
            let mut passed_zero = false;
            let mut passed_half = false;
            loop {
                let now = unsafe { ft.floating_time_consecutive() };
                if now < start && !passed_zero {
                    passed_zero = true;
                }
                if passed_zero && now >= U64_HALF && !passed_half {
                    passed_half = true;
                }
                if (passed_zero && now >= end) || (passed_half && now < U64_HALF) {
                    break;
                }
            }
        } else {
            // small wraparound 0 <= end << start <= u64::MAX
            while {
                let now = unsafe { ft.floating_time_consecutive() };
                now >= start || now < end
            } {
                // do nothing
            }
        }
    } else {
        // no wraparound: start <= end <= u64::MAX
        while {
            let now = unsafe { ft.floating_time_consecutive() };
            start <= now && now < end
        } {
            // do nothing
        }
    }

    #[cfg(feature = "bcm2835")]
    ::quartz::arch::arm1176::dsb()
}
