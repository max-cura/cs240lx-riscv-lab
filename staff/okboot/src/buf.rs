use crate::protocol::ReceiveError;
use core::fmt::Write;
use okboot_common::frame::{EncodeState, FrameEncoder, SliceBufferedEncoder};
use okboot_common::{EncodeMessageType, PREAMBLE_BYTES};
use serde::Serialize;
use thiserror::Error;

/// Buffer (not circular).
#[derive(Debug)]
pub struct ReceiveBuffer<'a> {
    underlying_storage: &'a mut [u8],
    cursor: usize,
}

impl<'a> ReceiveBuffer<'a> {
    pub fn new(underlying_storage: &'a mut [u8]) -> Self {
        Self {
            underlying_storage,
            cursor: 0,
        }
    }
    pub fn clear(&mut self) {
        self.cursor = 0;
        self.underlying_storage.fill(0);
    }
    pub fn push_u8(&mut self, b: u8) -> Result<(), ReceiveError> {
        if self.cursor >= self.underlying_storage.len() {
            Err(ReceiveError::BufferOverflow)
        } else {
            self.underlying_storage[self.cursor] = b;
            self.cursor += 1;
            Ok(())
        }
    }
    pub fn finalize(&mut self) -> &mut [u8] {
        let end = self.cursor;
        self.cursor = 0;
        &mut self.underlying_storage[..end]
    }
}

/// Circular buffer with FIFO semantics. Overlong writes will be truncated.
#[derive(Debug)]
pub struct TransmitBuffer<'a> {
    underlying_storage: &'a mut [u8],
    circle_begin: usize,
    circle_end: usize,
    /// Amount of data in active use by the buffer.
    circle_len: usize,
}

impl<'a> TransmitBuffer<'a> {
    pub fn new(underlying_storage: &'a mut [u8]) -> Self {
        Self {
            underlying_storage,
            circle_begin: 0,
            circle_end: 0,
            circle_len: 0,
        }
    }

    // Simple mutating utility methods -------------------------------------------------------------

    #[allow(dead_code)]
    pub fn clear(&mut self) {
        self.underlying_storage.iter_mut().for_each(|x| *x = 0);
        self.circle_begin = 0;
        self.circle_end = 0;
        self.circle_len = 0;
    }

    // Simple read-only utility methods ------------------------------------------------------------

    pub fn is_empty(&self) -> bool {
        self.circle_len == 0
    }

    pub fn remaining_space(&self) -> usize {
        self.underlying_storage.len() - self.circle_len
    }

    // Internals methods ---------------------------------------------------------------------------

    fn _wrapped_add(&self, a: usize, b: usize) -> (usize, bool) {
        // check: i+j<self.underlying_buffer.len()
        // ASSUME: i+j<usize::MAX since usize::MAX is more memory than we have on the Pi Zero
        let i = (a + b) % self.underlying_storage.len();
        (i, i < (a + b))
    }

    /// Write `byte` to the buffer at `offset`, returning the index of the byte immediately
    /// following the one at `offset`.
    fn _push_byte_at_unchecked(&mut self, offset: usize, byte: u8) -> usize {
        self.underlying_storage[offset] = byte;
        self._wrapped_add(offset, 1).0
    }

    /// Write `bytes` into the buffer at offset `offset`, wrapping if necessary. Will not change
    /// `circle_begin`, `circle_end`, or `circle_len`.
    fn _write_bytes_at_unchecked(
        &mut self,
        offset: usize,
        bytes: impl IntoIterator<Item = u8>,
    ) -> usize {
        let mut cursor = offset;
        for byte in bytes.into_iter() {
            cursor = self._push_byte_at_unchecked(cursor, byte);
        }
        cursor
    }

    // Methods for extending the buffer ------------------------------------------------------------

    pub fn push_byte(&mut self, byte: u8) -> bool {
        if self.circle_len == self.underlying_storage.len() {
            // full
            false
        } else {
            self.circle_end = self._push_byte_at_unchecked(self.circle_end, byte);
            // self.underlying_storage[self.circle_end] = byte;
            // self.circle_end += 1;
            self.circle_len += 1;

            true
        }
    }

    pub fn extend_from_slice(&mut self, src: &[u8]) -> bool {
        // LOGIC: circle_len <= storage.len
        if src.len() <= self.remaining_space() {
            for &b in src.iter() {
                // invariants: circle_end and circle_len get written properly by push byte, so this
                // is invariant-safe
                self.push_byte(b);
            }
            true
        } else {
            false
        }
    }

    pub fn reserve(&mut self, n_bytes: usize) -> Option<usize> {
        (n_bytes <= self.remaining_space()).then(|| {
            let v = self.circle_end;
            self.circle_end = self._wrapped_add(self.circle_end, n_bytes).0;
            self.circle_len += n_bytes;
            v
        })
    }

    // Methods for removing bytes at circle_begin --------------------------------------------------

    pub fn shift_byte(&mut self) -> Option<u8> {
        (self.circle_len > 0).then(|| {
            let b = self.underlying_storage[self.circle_begin];
            self.underlying_storage[self.circle_begin] = 0;
            self.circle_begin = self._wrapped_add(self.circle_begin, 1).0;
            self.circle_len -= 1;
            b
        })
    }
}

// Support for checkpointing, to allow rollback of buffer edits if a packet would fully overflow

#[derive(Debug, Copy, Clone)]
struct TransmitBufferCheckpoint {
    circle_begin: usize,
    circle_end: usize,
    circle_len: usize,
}

impl<'a> TransmitBuffer<'a> {
    fn checkpoint(&self) -> TransmitBufferCheckpoint {
        TransmitBufferCheckpoint {
            circle_begin: self.circle_begin,
            circle_end: self.circle_end,
            circle_len: self.circle_len,
        }
    }

    fn bytes_since_checkpoint(&self, cp: TransmitBufferCheckpoint) -> usize {
        let cp_end = cp.circle_end;
        if self.circle_end < cp_end {
            (self.underlying_storage.len() - cp_end) + self.circle_end
        } else {
            self.circle_end - cp_end
        }
    }

    fn restore(&mut self, from: TransmitBufferCheckpoint) {
        let TransmitBufferCheckpoint {
            circle_begin,
            circle_end,
            circle_len,
        } = from;
        self._write_bytes_at_unchecked(
            circle_end,
            core::iter::repeat_n(0, self.bytes_since_checkpoint(from)),
        );
        // self.underlying_storage[self.circle_end..circle_end].iter_mut().for_each(|x| *x = 0);
        self.circle_begin = circle_begin;
        self.circle_end = circle_end;
        self.circle_len = circle_len;
    }
}

pub struct FrameSink<'tb, 'be, 'px> {
    transmit_buffer: TransmitBuffer<'tb>,
    slice_buffered_encoder: SliceBufferedEncoder<'be>,
    postcard_buffer: Option<&'px mut [u8]>,
}
impl<'tb, 'be, 'px> FrameSink<'tb, 'be, 'px> {
    pub fn new(
        transmit_buffer: TransmitBuffer<'tb>,
        slice_buffered_encoder: SliceBufferedEncoder<'be>,
        postcard_buffer: &'px mut [u8],
    ) -> Self {
        Self {
            transmit_buffer,
            slice_buffered_encoder,
            postcard_buffer: Some(postcard_buffer),
        }
    }

    pub fn write_frame(&mut self) -> FrameWriter<'_, 'tb, '_> {
        FrameWriter::new(
            &mut self.transmit_buffer,
            self.slice_buffered_encoder.frame().unwrap(),
        )
    }

    pub fn buffer(&self) -> &TransmitBuffer {
        &self.transmit_buffer
    }

    pub fn buffer_mut<'a>(&'a mut self) -> &'a mut TransmitBuffer<'tb> {
        &mut self.transmit_buffer
    }
}

pub struct FrameWriter<'tbr, 'tb, 'fe> {
    transmit_buffer: &'tbr mut TransmitBuffer<'tb>,
    frame_encoder: FrameEncoder<'fe>,

    checkpoint: TransmitBufferCheckpoint,
    len_offset: usize,
    ok: bool,
    hasher: crc32fast::Hasher,
    fmt_bytes: usize,
}
impl<'tbr, 'tb, 'fe> FrameWriter<'tbr, 'tb, 'fe> {
    pub fn new(
        transmit_buffer: &'tbr mut TransmitBuffer<'tb>,
        frame_encoder: FrameEncoder<'fe>,
    ) -> Self {
        let checkpoint = transmit_buffer.checkpoint();
        let ok = transmit_buffer.extend_from_slice(&PREAMBLE_BYTES);
        let (len_offset, ok) = if ok {
            let lo = transmit_buffer.reserve(4);
            (lo.unwrap_or(0), lo.is_some())
        } else {
            (0, false)
        };
        Self {
            transmit_buffer,
            frame_encoder,
            checkpoint,
            len_offset,
            ok,
            hasher: crc32fast::Hasher::new(),
            fmt_bytes: 0,
        }
    }
    pub fn add_bytes(&mut self, bytes: &[u8]) {
        self.hasher.update(bytes);
        self._add_bytes_unhashed(bytes);
    }
    fn _add_bytes_unhashed(&mut self, bytes: &[u8]) {
        if !self.ok {
            return;
        }
        for byte in bytes.iter().copied() {
            if !self.ok {
                return;
            }
            match self.frame_encoder.write_u8(byte) {
                EncodeState::Buf(buf) => {
                    if self.ok {
                        self.ok = self.transmit_buffer.extend_from_slice(buf);
                    }
                }
                EncodeState::Pass => {}
            }
        }
    }
    pub fn write32(&mut self, x: u32) {
        self.add_bytes(&x.to_le_bytes());
    }
    pub fn finalize(mut self, payload_bytes: usize) -> bool {
        if self.ok {
            let hasher = core::mem::replace(&mut self.hasher, crc32fast::Hasher::new());
            // crc32 should be cobs-encoded, so run it through add_bytes
            self._add_bytes_unhashed(&hasher.finalize().to_le_bytes());
        }
        if self.ok {
            // finalize the packet, at last
            let buf = self.frame_encoder.finish();
            self.ok = self.transmit_buffer.extend_from_slice(buf);
        }
        if !self.ok {
            self.transmit_buffer.restore(self.checkpoint);
        } else {
            if let Some(len_bytes) = okboot_common::frame::encode_length(
                payload_bytes,
                // self.transmit_buffer.bytes_since_checkpoint(self.checkpoint) - 8,
            ) {
                self.transmit_buffer
                    ._write_bytes_at_unchecked(self.len_offset, len_bytes);
            } else {
                self.ok = false;
            }
        }
        self.ok
    }
    pub fn abort(self) {
        self.transmit_buffer.restore(self.checkpoint);
    }
    pub fn fmt_bytes(&self) -> usize {
        self.fmt_bytes
    }
}

impl<'tbr, 'tb, 'fe> Write for FrameWriter<'tbr, 'tb, 'fe> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.add_bytes(s.as_bytes());
        self.fmt_bytes += s.len();
        Ok(())
    }
}

#[derive(Debug, Error)]
pub enum SendError {
    #[error("encoding error: {0}")]
    Postcard(postcard::Error),
    #[error("error constructing frame encoder")]
    ConstructEncoder,
    #[error("insufficient space in buffer")]
    Truncated,
}

impl FrameSink<'_, '_, '_> {
    pub fn send<T: EncodeMessageType + Serialize>(&mut self, msg: &T) -> Result<(), SendError> {
        let px_buf = Option::take(&mut self.postcard_buffer).unwrap();
        let mut fw = FrameWriter::new(
            &mut self.transmit_buffer,
            self.slice_buffered_encoder
                .frame()
                .ok_or(SendError::ConstructEncoder)?,
        );
        fw.write32(<T as EncodeMessageType>::TYPE as u32);
        let r = match postcard::to_slice(msg, px_buf) {
            Ok(buf) => {
                fw.add_bytes(buf);
                if fw.finalize(buf.len()) {
                    Ok(())
                } else {
                    Err(SendError::Truncated)
                }
            }
            Err(e) => {
                fw.abort();
                Err(SendError::Postcard(e))
            }
        };
        self.postcard_buffer = Some(px_buf);
        r
    }
}

pub fn send<T: EncodeMessageType + Serialize>(
    fs: &mut FrameSink,
    msg: &T,
) -> Result<(), SendError> {
    fs.send(msg)
}

/// Legacy println
pub mod legacy_compat {
    use super::{FrameSink, TransmitBuffer, TransmitBufferCheckpoint};
    use core::fmt::Write;

    pub trait LegacyPrintString<'tb> {
        fn legacy_print_string(&mut self) -> Writer<'_, 'tb>;
    }
    impl<'tb, 'be, 'px> LegacyPrintString<'tb> for FrameSink<'tb, 'be, 'px> {
        fn legacy_print_string(&mut self) -> Writer<'_, 'tb> {
            let checkpoint = self.transmit_buffer.checkpoint();
            let mut ok = self.transmit_buffer.extend_from_slice(&u32::to_le_bytes(
                okboot_common::su_boot::Command::PrintString as u32,
            ));
            let len_offset = if ok {
                if let Some(len_offset) = self.transmit_buffer.reserve(4) {
                    len_offset
                } else {
                    ok = false;
                    0
                }
            } else {
                0
            };
            Writer {
                transmit_buffer: &mut self.transmit_buffer,
                checkpoint,
                len_offset,
                ok,
            }
        }
    }

    pub struct Writer<'tbr, 'tb> {
        transmit_buffer: &'tbr mut TransmitBuffer<'tb>,
        checkpoint: TransmitBufferCheckpoint,
        len_offset: usize,
        ok: bool,
    }
    impl<'a, 'b> Writer<'a, 'b> {
        pub fn finalize(self) -> bool {
            if !self.ok {
                self.transmit_buffer.restore(self.checkpoint);
            } else {
                self.transmit_buffer._write_bytes_at_unchecked(
                    self.len_offset,
                    (self.transmit_buffer.bytes_since_checkpoint(self.checkpoint) as u32 - 8)
                        .to_le_bytes(),
                );
            }
            self.ok
        }
    }
    impl<'a, 'b> Write for Writer<'a, 'b> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            if self.ok {
                self.ok = self.transmit_buffer.extend_from_slice(s.as_bytes());
            }
            Ok(())
        }
    }

    #[macro_export]
    macro_rules! legacy_print_string {
    ($tx_buf:expr, $($args:tt)*) => {
        // $w is expected to be transmit buffer
        #[allow(unused_imports)]
        use ::core::fmt::Write as _;
        let txbuf : &mut $crate::buf::FrameSink = $tx_buf;
        let mut writer = $crate::buf::legacy_compat::LegacyPrintString::legacy_print_string(txbuf);
        let _ = ::core::writeln!(
            writer,
            $($args)*
        );
        writer.finalize();
    }
    }
}

/// V2 println
pub mod print {
    #[macro_export]
    macro_rules! rpc_println {
    ($fs:expr, $($args:tt)*) => {
        #[allow(unused_imports)]
        use ::core::fmt::Write as _;
        let fs: &mut $crate::buf::FrameSink = $fs;
        let mut writer = fs.write_frame();
        writer.write32(::okboot_common::MessageType::PrintString as u32);
        ::core::writeln!(&mut writer, $($args)*).unwrap();
        let fmt_bytes = writer.fmt_bytes();
        writer.finalize(fmt_bytes);
    }
    }
}
