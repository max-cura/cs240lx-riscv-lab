use crate::frame::COBS_SENTINEL;
use core::borrow::BorrowMut;
use core::fmt::Debug;

pub fn encode_length(length: usize) -> Option<[u8; 4]> {
    if (length & !0xff_ffff) != 0 {
        None
    } else {
        let splits = [
            (length & 0x3f) as u8,
            ((length & 0xfc0) >> 6) as u8,
            ((length & 0x3_f000) >> 12) as u8,
            ((length & 0xfc_0000) >> 18) as u8,
        ]
        .map(|x| x | 0xc0);
        Some(splits)
    }
}

#[derive(Debug)]
pub struct BufferedEncoder<T: BorrowMut<[u8]> + Debug> {
    buffer: T,
    xor: u8,
}
impl<T: BorrowMut<[u8]> + Debug> BufferedEncoder<T> {
    pub fn with_buffer_xor(buffer: T, xor: u8) -> Self {
        Self { buffer, xor }
    }
    pub fn frame(&mut self) -> Option<FrameEncoder> {
        FrameEncoder::with_buffer_xor(self.buffer.borrow_mut(), self.xor)
    }
}

pub type SliceBufferedEncoder<'be> = BufferedEncoder<&'be mut [u8]>;

#[derive(Debug)]
pub enum EncodeState<'a> {
    Buf(&'a [u8]),
    Pass,
}
#[derive(Debug)]
pub struct FrameEncoder<'a> {
    inner: FrameEncoderInner<'a>,
    xor: u8,
}
impl<'a> FrameEncoder<'a> {
    pub fn with_buffer_xor(underlying_buffer: &'a mut [u8], xor: u8) -> Option<Self> {
        Some(Self {
            inner: FrameEncoderInner::with_buffer(underlying_buffer)?,
            xor,
        })
    }
    pub fn write_u8(&mut self, x: u8) -> EncodeState {
        match self.inner.write_u8(x) {
            EncodeStateInner::Buf(b) => {
                b.iter_mut().for_each(|x| *x ^= self.xor);
                EncodeState::Buf(&b[..])
            }
            EncodeStateInner::Pass => EncodeState::Pass,
        }
    }
    pub fn finish(self) -> &'a [u8] {
        let b = self.inner.finish();
        b.iter_mut().for_each(|x| *x ^= self.xor);
        &b[..]
    }
}

#[derive(Debug)]
pub enum EncodeStateInner<'a> {
    Buf(&'a mut [u8]),
    Pass,
}
#[derive(Debug)]
pub struct FrameEncoderInner<'a> {
    buf: &'a mut [u8],
    cursor: usize,
}
impl<'a> FrameEncoderInner<'a> {
    pub fn with_buffer(underlying_buffer: &'a mut [u8]) -> Option<Self> {
        (underlying_buffer.len() == 255).then(|| Self {
            buf: underlying_buffer,
            cursor: 1,
        })
    }
    pub fn write_u8(&mut self, x: u8) -> EncodeStateInner {
        if self.cursor == 254 {
            // 0 [1 ... 254] 255
            //           ^- we are here
            // Case 1:
            //  x = 0 -> buf[0] = fe
            //  x ≠ 0 -> buf[0] = ff, buf[fe]=x
            if x == COBS_SENTINEL {
                self.buf[0] = 0xfe;
                self.cursor = 1;
                EncodeStateInner::Buf(&mut self.buf[0..254])
            } else {
                self.buf[0] = 0xff;
                self.buf[0xfe] = x;
                self.cursor = 1;
                EncodeStateInner::Buf(&mut self.buf[0..255])
            }
        } else if x != COBS_SENTINEL {
            self.buf[self.cursor] = x;
            self.cursor += 1;
            EncodeStateInner::Pass
        } else {
            self.buf[0] = self.cursor as u8;
            let saved_cursor = self.cursor;
            self.cursor = 1;
            EncodeStateInner::Buf(&mut self.buf[0..saved_cursor])
        }
    }

    pub fn finish(mut self) -> &'a mut [u8] {
        self.buf[self.cursor] = COBS_SENTINEL;
        self.buf[0] = self.cursor as u8;
        self.cursor += 1;
        &mut self.buf[0..self.cursor]
    }
}
