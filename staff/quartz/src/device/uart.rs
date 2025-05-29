pub trait Uart {
    /// Block until TX buffer is flushed, and has been sent over the wire.
    fn flush_tx(&self);

    /// Set a new baud rate.
    fn set_baud_rate(&self, new_baud_rate: u32);

    /// Return true if the FIFO is not full (and more bytes can be written to it).
    unsafe fn can_write_unchecked(&self) -> bool;
    /// Write a byte to the UART.
    unsafe fn write_unchecked(&self, byte: u8);
    /// Return true if data is available to be read.
    unsafe fn data_available_unchecked(&self) -> bool;
    /// Read a byte from the input FIFO.
    unsafe fn read_unchecked(&self) -> u8;

    /// The type being wrapped; in case access is required to the underlying UART object.
    type Inner;
    /// Get access to some kind of underlying object.
    fn raw(&self) -> &Self::Inner;

    /// Wrapper function over [`can_write_unchecked`] and [`write_unchecked`] that will write out
    /// the bytestring `bytes` to the FIFO, ensuring that teh FIFO is not overrun in the process.
    unsafe fn write_bytes_blocking_unchecked(&self, bytes: &[u8]) {
        for byte in bytes.iter() {
            while !unsafe { self.can_write_unchecked() } {}
            unsafe { self.write_unchecked(*byte) }
        }
    }
}
