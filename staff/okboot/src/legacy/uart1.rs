// use bcm2835_lpa::UART1;
use quartz::device::uart::Uart;

pub fn uart1_write8<U: Uart>(uart: &U, x: u8) {
    // dsb();

    // while !uart1_device.stat().read().tx_ready().bit_is_set() {}
    // uart1_device.io().write(|w| unsafe { w.data().bits(x) });

    unsafe {
        while !uart.can_write_unchecked() {}
        uart.write_unchecked(x);
    }

    // dsb();
}

pub fn uart1_write_bytes<U: Uart>(uart: &U, x: &[u8]) {
    for &b in x.iter() {
        uart1_write8(uart, b)
    }
}

pub fn uart1_write32<U: Uart>(uart: &U, x: u32) {
    uart1_write_bytes(uart, &u32::to_le_bytes(x));
}

pub fn uart1_read32_blocking<U: Uart>(uart: &U) -> u32 {
    let mut buf = [0; 4];
    for i in 0..4 {
        buf[i] = uart1_read8_blocking(uart);
    }
    u32::from_le_bytes(buf)
}

pub fn uart1_read8_nb<U: Uart>(uart: &U) -> Option<u8> {
    // dsb();

    // let r = if uart1_device.stat().read().data_ready().bit_is_set() {
    //     Some(uart1_device.io().read().data().bits())
    // } else {
    //     None
    // };
    let r = unsafe {
        if uart.data_available_unchecked() {
            Some(uart.read_unchecked())
        } else {
            None
        }
    };

    // dsb();
    r
}

pub fn uart1_read8_blocking<U: Uart>(uart: &U) -> u8 {
    // dsb();

    // while !uart1_device.stat().read().data_ready().bit_is_set() {}
    // let b = uart1_device.io().read().data().bits();

    let b = unsafe {
        while !uart.data_available_unchecked() {}
        uart.read_unchecked()
    };

    // dsb();

    b
}

pub fn uart1_flush_tx<U: Uart>(uart: &U) {
    // dsb();

    // while !uart1_device.stat().read().tx_empty().bit_is_set() {}
    uart.flush_tx();

    // dsb();
}
