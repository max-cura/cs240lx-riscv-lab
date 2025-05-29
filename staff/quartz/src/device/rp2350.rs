use rp235x_pac::{RESETS, SIO, UART0};

use super::{timing::FloatingTimer, uart::Uart};

/// Implements [`FloatingTimer`] using the RISC-V timer; [`init_riscv_timer_from_clk_ref`] must
/// have been called to initialize the timer before using `RiscvTimer`.
pub struct RiscvTimer;

impl RiscvTimer {
    pub const fn new() -> Self {
        Self
    }
}

impl FloatingTimer for RiscvTimer {
    fn floating_time(&self) -> u64 {
        let sio = unsafe { SIO::steal() };
        // Needs to be a loop because it's possible that one could have:
        //   read mtimeh
        //   (timer wraps around), incrementing mtimeh
        //   read mtime
        // with the result being that the timer appears to go backwards. This is bad (breaks
        // monotonicity), so we do a loop and make sure that the high bits of the timer value
        // haven't changed.
        //
        // XXX: hazard3 doesn't implement the time/timeh CSR
        let mut out_h: u32 = sio.mtimeh().read().bits();
        let mut out_l: u32;
        loop {
            out_l = sio.mtime().read().bits();
            let tmp = sio.mtimeh().read().bits();
            if tmp == out_h {
                break;
            }
            out_h = tmp;
        }
        (u64::from(out_h) << 32) | u64::from(out_l)
    }
}

pub struct PL011Uart<'u> {
    // Annoyingly, the SVD that rp235x is based on doesn't unify the RegisterBlock for UART0/UART1.
    // We could fix this here, but that would be a PITA.
    uart: &'u UART0,
}
impl<'u> PL011Uart<'u> {
    /// Create a `PL011Uart` without running the UART initialization code.
    pub fn new(uart: &'u UART0) -> Self {
        Self { uart }
    }

    /// Initialize the PL011 with the given baud rate; note that this is parameterized by the
    /// CLK_PERI frequency, since this is what the UART's clocking is based on.
    pub fn init(baud_rate: u32, peripheral_clock: u32, uart: &'u UART0, resets: &RESETS) -> Self {
        // 1. Deassert UART reset and wait for it to take effect
        resets.reset().modify(|_, w| w.uart0().clear_bit());
        while resets.reset_done().read().uart0().bit_is_clear() {}

        // 2. Set baud rate; note that baud rate changes seem to require a write to LCR_H
        // afterwards in certain cases
        let baud_rate_div = ((peripheral_clock / baud_rate) * 8) + 1;
        let baud_rate_int = baud_rate_div >> 7;
        let (baud_rate_int, baud_rate_frac) = if baud_rate_int == 0 {
            (1, 0)
        } else if baud_rate_int >= u16::MAX as _ {
            (u16::MAX, 0)
        } else {
            (baud_rate_int as u16, ((baud_rate_div & 0x7f) >> 1) as u8)
        };
        unsafe {
            uart.uartibrd()
                .write_with_zero(|w| w.baud_divint().bits(baud_rate_int));
            uart.uartfbrd()
                .write_with_zero(|w| w.baud_divfrac().bits(baud_rate_frac));
        }

        // 3. Set up LCR_H
        unsafe {
            // fen : FIFO enable
            // wlen=3 : 8-bit symbols
            uart.uartlcr_h()
                .write_with_zero(|w| w.fen().set_bit().wlen().bits(3));
        }
        // 4. Set enable bits in CR
        unsafe {
            uart.uartcr()
                .write_with_zero(|w| w.uarten().set_bit().txe().set_bit().rxe().set_bit());
        }
        // // 5. Seems like we need to turn on DMACR stuff??
        // unsafe {
        //     uart.uartdmacr()
        //         .write_with_zero(|w| w.rxdmae().set_bit().txdmae().set_bit());
        // }

        Self { uart }
    }
}
impl Uart for PL011Uart<'_> {
    fn flush_tx(&self) {
        loop {
            let fr = self.uart.uartfr().read();
            // Need to wait until FIFO is empty AND shift register has been cleared.
            if fr.txfe().bit_is_set() && fr.busy().bit_is_clear() {
                break;
            }
        }
    }

    // XXX: this assumes clk_peri = 150MHz
    fn set_baud_rate(&self, new_baud_rate: u32) {
        let baud_rate_div = ((150_000_000 / new_baud_rate) * 8) + 1;
        let baud_rate_int = baud_rate_div >> 7;
        let (baud_rate_int, baud_rate_frac) = if baud_rate_int == 0 {
            (1, 0)
        } else if baud_rate_int >= u16::MAX as _ {
            (u16::MAX, 0)
        } else {
            (baud_rate_int as u16, ((baud_rate_div & 0x7f) >> 1) as u8)
        };
        unsafe {
            self.uart
                .uartibrd()
                .write_with_zero(|w| w.baud_divint().bits(baud_rate_int));
            self.uart
                .uartfbrd()
                .write_with_zero(|w| w.baud_divfrac().bits(baud_rate_frac));
        }
        self.uart.uartlcr_h().modify(|_, w| w);
    }

    unsafe fn can_write_unchecked(&self) -> bool {
        // txff = transmit FIFO full
        self.uart.uartfr().read().txff().bit_is_clear()
    }

    unsafe fn write_unchecked(&self, byte: u8) {
        unsafe { self.uart.uartdr().write_with_zero(|w| w.data().bits(byte)) }
    }

    type Inner = UART0;
    fn raw(&self) -> &Self::Inner {
        &self.uart
    }

    unsafe fn data_available_unchecked(&self) -> bool {
        // rxfe = receive FIFO empty
        self.uart.uartfr().read().rxfe().bit_is_clear()
    }

    unsafe fn read_unchecked(&self) -> u8 {
        self.uart.uartdr().read().data().bits()
    }
}
