//! Message frame format:
//! ```txt
//! | preamble | length | COBS FRAME                                      | TERMINATING      |
//! | preamble | length | CONTENT CRC IS CALCULATED ON   | crc            | DELIMITER        |
//! | preamble | length | type | payload                 | crc            | 0                |
//!   +0:4     | +4:4   | +8:4   +12:(length)     +(12+length):4   +(12+length+4):1
//! ```

/// Used internally, by [`decode`] and [`encode`].
/// This is NOT the COBS xor value.
const COBS_SENTINEL: u8 = 0;

/// Provides functionality for picking out messages from a byte stream
mod decode;
pub use decode::{CobsError, FrameError, FrameHeader, FrameLayer, FrameOutput, PreambleError};

/// Provides functionality for writing out a stream of COBS-stuffed data.
mod encode;
pub use encode::{encode_length, BufferedEncoder, EncodeState, FrameEncoder, SliceBufferedEncoder};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::decode::{CobsDecoder, CobsState};
    use crate::frame::encode::encode_length;
    use crate::{MessageType, COBS_XOR, PREAMBLE_BYTES};
    use rand::RngCore;
    use std::prelude::rust_2021::*;
    use std::vec;

    /// Test that COBS works properly
    #[test]
    fn test_cobs() {
        let mut buf = [0; 255];
        let mut enc = FrameEncoder::with_buffer_xor(&mut buf, COBS_XOR)
            .expect("should be able to create frame encoder");
        let data: Vec<u8> = ((0u8..255).chain(0..255)).collect();
        let mut out = vec![];
        for i in &data {
            match enc.write_u8(*i) {
                EncodeState::Buf(b) => {
                    out.extend_from_slice(b);
                }
                EncodeState::Pass => {}
            }
        }
        out.extend_from_slice(enc.finish());
        assert!(out.iter().any(|&b| b == COBS_XOR));

        let mut dec = CobsDecoder::new(COBS_XOR);
        let mut j = 0;
        let mut did_finish = false;
        for &i in &out {
            match dec.poll(i).expect("error during COBS decoding") {
                CobsState::Skip => {}
                CobsState::Byte(b) => {
                    assert_eq!(data[j], b, "mismatch at position {j}");
                    j += 1;
                }
                CobsState::Finished => {
                    did_finish = true;
                    assert_eq!(
                        j,
                        data.len(),
                        "decoded data has unexpected length: expected {} got {j}",
                        data.len()
                    );
                }
            }
        }
        assert!(did_finish, "did not finish");
    }

    /// Test that BufferedEncoder resets properly
    #[test]
    fn test_buffered_encoder() {
        fn single(data: &[u8]) -> Vec<u8> {
            let mut buf = [0; 255];
            let mut enc = FrameEncoder::with_buffer_xor(&mut buf, COBS_XOR)
                .expect("should be able to create frame encoder");
            let mut out = vec![];
            for k in data.iter() {
                match enc.write_u8(*k) {
                    EncodeState::Buf(b) => {
                        out.extend_from_slice(b);
                    }
                    EncodeState::Pass => {}
                }
            }
            out.extend_from_slice(enc.finish());
            out
        }
        let mut data1 = vec![0; 1024];
        let mut data2 = vec![0; 1024];
        rand::thread_rng().fill_bytes(&mut data1);
        rand::thread_rng().fill_bytes(&mut data2);
        let enc1_single = single(&data1);
        let enc2_single = single(&data2);

        let mut buf = [0u8; 255];
        let mut enc = BufferedEncoder::with_buffer_xor(buf.as_mut(), COBS_XOR);
        let mut borrowed = |data: &[u8]| -> Vec<u8> {
            let mut enc = enc.frame().expect("should be able to create frame encoder");
            let mut out = vec![];
            for b in data {
                match enc.write_u8(*b) {
                    EncodeState::Buf(b) => {
                        out.extend_from_slice(b);
                    }
                    EncodeState::Pass => {}
                }
            }
            out.extend_from_slice(enc.finish());
            out
        };
        let enc1_borrow = borrowed(&data1);
        let enc2_borrow = borrowed(&data2);

        assert_eq!(enc1_single, enc1_borrow);
        assert_eq!(enc2_single, enc2_borrow);
    }

    /// Test that the full decode pipeline works
    #[test]
    fn test_decode() {
        let mut payload = vec![0u8; 0];
        rand::thread_rng().fill_bytes(&mut payload);
        let mut bytes = vec![];
        {
            bytes.extend_from_slice(&PREAMBLE_BYTES);
            let l = encode_length(payload.len()).expect("failed to encode payload length");
            std::eprintln!("len: {l:?}");
            bytes.extend_from_slice(&l);
            let mut input = vec![];
            input.extend_from_slice(&u32::to_le_bytes(MessageType::PrintString as u32));
            input.extend_from_slice(&payload);
            let crc = crc32fast::hash(&input);
            input.extend_from_slice(&u32::to_le_bytes(crc));
            let mut buf = [0; 255];
            let mut frame = FrameEncoder::with_buffer_xor(&mut buf, COBS_XOR)
                .expect("should be able to create frame encoder");
            for &i in input.iter() {
                if let EncodeState::Buf(b) = frame.write_u8(i) {
                    bytes.extend_from_slice(b);
                }
            }
            bytes.extend_from_slice(frame.finish());
            // 3 in the preamble, and 1 at the end
            assert_eq!(bytes.iter().filter(|x| **x == COBS_XOR).count(), 4);
        }

        let mut dec = FrameLayer::new(COBS_XOR);
        let mut hdr = None;
        let mut j = 0;
        let mut did_finish = false;
        for &i in bytes.iter() {
            match dec.feed(i).expect("error during frame decoding") {
                FrameOutput::Skip => {}
                FrameOutput::Header(h) => {
                    let prev = hdr.replace(h);
                    assert!(
                        prev.is_none(),
                        "detected second header; original {:?} second: {h:?}",
                        prev.unwrap()
                    );
                }
                FrameOutput::Payload(b) => {
                    assert_eq!(
                        payload[j], b,
                        "mismatch at position {j}: expected {} got {b}",
                        payload[j]
                    );
                    j += 1;
                }
                FrameOutput::Finished => {
                    did_finish = true;
                }
                FrameOutput::Legacy => {
                    panic!("entered legacy mode");
                }
                FrameOutput::LegacyPrintStringByte(_, _) => {
                    panic!("entered legacy mode print-string");
                }
            }
        }
        assert!(did_finish, "did not finish");
    }
}
