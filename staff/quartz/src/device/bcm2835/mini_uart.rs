use crate::arch::arm1176::dsb;
use bcm2835_lpa::{AUX, GPIO, UART1};

const MINI_UART_CLOCK_RATE: u32 = 250_000_000;

/// Calculate a value for the Mini UART clock divider from the desired baud rate.
pub const fn baud_to_clock_divider(baud_rate: u32) -> u16 {
    ((MINI_UART_CLOCK_RATE / (8 * baud_rate)) - 1) as u16
}

pub fn muart1_init(gpio: &GPIO, aux: &AUX, uart: &UART1, clock_divider: u16) {
    // TODO: check interrupts disabled

    dsb();

    gpio.gpfsel1()
        .modify(|_, w| w.fsel14().txd1().fsel15().rxd1());

    dsb();

    aux.enables().modify(|_, w| w.uart_1().set_bit());

    dsb();

    uart.cntl()
        .write(|w| w.tx_enable().clear_bit().rx_enable().clear_bit());

    unsafe {
        uart.ier().write_with_zero(|w| {
            // docs are a bit screwy-I also don't completely trust bcm2835-lpa here, either
            // however, we can just w.bits(0) to disable all interrupts so
            w.bits(0)
        })
    };
    __uart1_clear_fifos_unguarded(uart);
    uart.baud().write(|w| unsafe { w.bits(clock_divider) });
    uart.lcr().write(|w| {
        w.data_size()._8bit()
        // .break_().clear_bit()
        // .dlab().clear_bit()
    });
    uart.mcr().write(|w| w.rts().clear_bit());
    // uart.cntl()
    // .modify(|_, w| w.cts_enable().clear_bit().rts_enable().clear_bit());
    uart.cntl()
        .modify(|_, w| w.tx_enable().set_bit().rx_enable().set_bit());

    dsb();
}

fn __uart1_flush_tx_unguarded(uart: &UART1) {
    while uart.stat().read().tx_done().bit_is_clear() {}
}

fn __uart1_clear_fifos_unguarded(uart: &UART1) {
    // clear FIFOs
    uart.iir().modify(|_, w| {
        // names are wrong - functionality on writing is different from
        // functionality on read.
        // writing 11 to bits 3:2 will clear both FIFOs
        w.tx_ready().set_bit().data_ready().set_bit()
    });
}

pub fn __uart1_clear_fifos(uart: &UART1) {
    dsb();
    __uart1_clear_fifos_unguarded(uart);
    dsb();
}

pub fn mini_uart1_set_clock(uart: &UART1, new_divider: u16) -> bool {
    dsb();
    __uart1_flush_tx_unguarded(uart);
    let old_clock_divider = uart.baud().read().bits();
    uart.cntl()
        .modify(|_, w| w.tx_enable().clear_bit().rx_enable().clear_bit());
    uart.baud().write(|w| unsafe { w.bits(new_divider) });
    let succeeded = uart.baud().read().bits() == new_divider;
    if !succeeded {
        uart.baud().write(|w| unsafe { w.bits(old_clock_divider) })
    }
    let _ = uart.lsr().read().bits();
    __uart1_clear_fifos_unguarded(uart);
    uart.cntl()
        .modify(|_, w| w.tx_enable().set_bit().rx_enable().set_bit());
    dsb();
    succeeded
}

pub fn mini_uart1_flush_tx(uart: &UART1) {
    dsb();
    // actually for real tx_empty
    while !uart.stat().read().tx_done().bit_is_set() {}
    dsb();
}
