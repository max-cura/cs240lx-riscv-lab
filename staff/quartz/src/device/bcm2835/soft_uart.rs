use crate::arch::arm1176::{cycle, dsb};
use bcm2835_lpa::Peripherals;
use core::fmt::Write;

pub struct Uart8n1<const TX: u8, const RX: u8> {
    cycles_per_bit: u32,
}
fn baud_to_cycles(baud: u32) -> u32 {
    // cycles -> 250MHz
    700_000_000 / baud
}
impl<const TX: u8, const RX: u8> Uart8n1<TX, RX> {
    pub fn new(baud: u32) -> Self {
        Self {
            cycles_per_bit: baud_to_cycles(baud),
        }
    }
    pub fn set_baud_rate(&mut self, baud_rate: u32) {
        self.cycles_per_bit = baud_to_cycles(baud_rate);
    }
    pub fn warm_up(&self, peri: &Peripherals) {
        unsafe {
            peri.GPIO.gpset0().write_with_zero(|w| w.bits(1 << TX));
        }
        let start = cycle::ccr_read();
        while cycle::ccr_read() < start + 2 * self.cycles_per_bit {}
    }
    pub fn write_byte(&self, mut byte: u8, peri: &Peripherals) {
        let start = cycle::ccr_read();
        unsafe {
            dsb();
            peri.GPIO.gpclr0().write_with_zero(|w| w.bits(1 << TX));
            dsb();
            while cycle::ccr_read() < start + self.cycles_per_bit {}
        }
        let mut last_bit = 0;
        for i in 2..10 {
            let bit = byte & 1;
            byte >>= 1;
            if bit != last_bit {
                dsb();
                if bit == 0 {
                    unsafe { peri.GPIO.gpclr0().write_with_zero(|w| w.bits(1 << TX)) };
                } else {
                    unsafe { peri.GPIO.gpset0().write_with_zero(|w| w.bits(1 << TX)) };
                }
                dsb();
            }
            last_bit = bit;
            while cycle::ccr_read() < start + i * self.cycles_per_bit {}
        }
        unsafe {
            dsb();
            peri.GPIO.gpset0().write_with_zero(|w| w.bits(1 << TX));
            dsb();
            while cycle::ccr_read() < start + 10 * self.cycles_per_bit {}
        }
    }
}
impl<const TX: u8, const RX: u8> Write for Uart8n1<TX, RX> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let peri = unsafe { Peripherals::steal() };
        for b in s.bytes() {
            self.write_byte(b, &peri);
        }
        Ok(())
    }
}
