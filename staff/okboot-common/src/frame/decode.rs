use crate::frame::COBS_SENTINEL;
use crate::MessageType;
use core::hash::Hasher;
use core::intrinsics::{likely, unlikely};
use thiserror::Error;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum PreambleState {
    Initial,
    Preamble1,
    Preamble2,
    Preamble3,
    LegacyPutProgramInfo1,
    LegacyPutProgramInfo2,
    LegacyPutProgramInfo3,
    LegacyPutString1,
    LegacyPutString2,
    LegacyPutString3,
    LegacyPutStringLength0,
    LegacyPutStringLength1([u8; 1]),
    LegacyPutStringLength2([u8; 2]),
    LegacyPutStringLength3([u8; 3]),
}
pub enum PreambleStatus {
    Skip,
    Preamble,
    LegacyPutProgramInfo,
    LegacyPutString(u32),
}
#[derive(Clone, Copy, Debug, Error)]
#[error("Found byte {wrong_byte:02x} in state {state:?}")]
pub struct PreambleError {
    state: PreambleState,
    wrong_byte: u8,
}
#[derive(Debug)]
struct PreambleDecoder {
    state: PreambleState,
}
impl PreambleDecoder {
    pub fn new() -> Self {
        Self {
            state: PreambleState::Initial,
        }
    }
    fn reset(&mut self) {
        self.state = PreambleState::Initial
    }
    fn feed(&mut self, byte: u8) -> Result<PreambleStatus, PreambleError> {
        let new_state = match (self.state, byte) {
            (PreambleState::Initial, 0x55) => PreambleState::Preamble1,
            (PreambleState::Preamble1, 0x55) => PreambleState::Preamble2,
            (PreambleState::Preamble2, 0x55) => PreambleState::Preamble3,
            (PreambleState::Preamble3, 0x5e) => {
                self.state = PreambleState::Initial;
                return Ok(PreambleStatus::Preamble);
            }
            (PreambleState::Preamble3, 0x55) => PreambleState::Preamble3,
            (PreambleState::Initial, 0x44) => PreambleState::LegacyPutProgramInfo1,
            (PreambleState::LegacyPutProgramInfo1, 0x44) => PreambleState::LegacyPutProgramInfo2,
            (PreambleState::LegacyPutProgramInfo2, 0x33) => PreambleState::LegacyPutProgramInfo3,
            (PreambleState::LegacyPutProgramInfo3, 0x33) => {
                self.state = PreambleState::Initial;
                return Ok(PreambleStatus::LegacyPutProgramInfo);
            }
            (PreambleState::Initial, 0xee) => PreambleState::LegacyPutString1,
            (PreambleState::LegacyPutString1, 0xee) => PreambleState::LegacyPutString2,
            (PreambleState::LegacyPutString2, 0xdd) => PreambleState::LegacyPutString3,
            (PreambleState::LegacyPutString3, 0xdd) => PreambleState::LegacyPutStringLength0,
            (PreambleState::LegacyPutStringLength0, x) => {
                PreambleState::LegacyPutStringLength1([x])
            }
            (PreambleState::LegacyPutStringLength1([a]), x) => {
                PreambleState::LegacyPutStringLength2([a, x])
            }
            (PreambleState::LegacyPutStringLength2([a, b]), x) => {
                PreambleState::LegacyPutStringLength3([a, b, x])
            }
            (PreambleState::LegacyPutStringLength3([a, b, c]), x) => {
                self.state = PreambleState::Initial;
                return Ok(PreambleStatus::LegacyPutString(u32::from_le_bytes([
                    a, b, c, x,
                ])));
            }
            (state, wrong_byte) => {
                self.state = PreambleState::Initial;
                return Err(PreambleError { state, wrong_byte });
            }
        };
        self.state = new_state;
        Ok(PreambleStatus::Skip)
    }
}
/// Decodes the preamble from a message stream.
#[derive(Debug)]
pub struct PreambleLayer {
    preamble_decoder: PreambleDecoder,
}
impl PreambleLayer {
    pub fn new() -> Self {
        Self {
            preamble_decoder: PreambleDecoder::new(),
        }
    }
    pub fn reset(&mut self) {
        self.preamble_decoder.reset();
    }
    /// Returns `Ok(true)` if `byte` was the last byte of a complete and valid preamble.
    /// Returns `Ok(false)` if `byte` is the next byte of a valid preamble.
    /// Returns `Err` if `byte` is not part of a valid preamble.
    pub fn poll(&mut self, byte: u8) -> Result<PreambleStatus, PreambleError> {
        self.preamble_decoder.feed(byte)
    }
}

// This can only occur if there was a premature sentinel byte, or no sentinel byte was found.
#[derive(Debug, Clone, Copy, Error)]
#[error("At position {frame_loc} found byte {found:02x}, with last_jump={last_jump} and bytes_since_last_jump={bytes_since_last_jump}")]
pub struct CobsError {
    frame_loc: usize,
    bytes_since_last_jump: usize,
    last_jump: usize,
    found: u8,
}
#[derive(Debug)]
pub enum CobsState {
    Skip,
    Byte(u8),
    Finished,
}
#[derive(Debug)]
pub struct CobsDecoder {
    frame_loc: usize,
    bytes_since_last_jump: usize,
    last_jump: usize,
    xor: u8,
}
impl CobsDecoder {
    pub fn new(xor: u8) -> Self {
        Self {
            frame_loc: 0,
            bytes_since_last_jump: 0,
            last_jump: 0,
            xor,
        }
    }
    fn reset(&mut self) {
        self.frame_loc = 0;
        self.bytes_since_last_jump = 0;
        self.last_jump = 0;
    }
    fn error(&self, found: u8) -> CobsError {
        CobsError {
            frame_loc: self.frame_loc,
            bytes_since_last_jump: self.bytes_since_last_jump,
            last_jump: self.last_jump,
            found,
        }
    }
    pub fn poll(&mut self, byte: u8) -> Result<CobsState, CobsError> {
        let byte = byte ^ self.xor;
        if self.last_jump == 0 {
            // CASE: packet start

            // Byte should not be zero
            if unlikely(byte == COBS_SENTINEL) {
                return Err(self.error(byte));
            }
            // Impossible case
            if unlikely(self.bytes_since_last_jump != self.last_jump) {
                return Err(self.error(byte));
            }

            self.last_jump = byte as usize;
            self.bytes_since_last_jump = 0;
            return Ok(CobsState::Skip);
        }
        // Okay, tick forward one
        self.bytes_since_last_jump += 1;
        if likely(self.bytes_since_last_jump < self.last_jump) {
            // CASE: in between jumps
            if unlikely(byte == COBS_SENTINEL) {
                Err(self.error(byte))
            } else {
                Ok(CobsState::Byte(byte))
            }
        } else {
            // CASES:
            //  b = 0 -> finished
            //  b ≠ 0, LJ<255 -> byte
            //  b ≠ 0, LJ=255 -> skip
            let prev_jump = self.last_jump;
            self.last_jump = byte as usize;
            self.bytes_since_last_jump = 0;
            if unlikely(byte == 0) {
                self.reset();
                Ok(CobsState::Finished)
            } else {
                if likely(prev_jump < 0xff) {
                    Ok(CobsState::Byte(0))
                } else {
                    Ok(CobsState::Skip)
                }
            }
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct FrameHeader {
    pub message_type: MessageType,
    pub payload_len: usize,
}
#[derive(Debug, Copy, Clone, Error)]
pub enum FrameError {
    #[error("Invalid message type: {0}")]
    InvalidType(u32),
    #[error("Invalid CRC: calculated {0}, expected {1}")]
    InvalidCRC(u32, u32),
    #[error("Error during COBS decoding: {0}")]
    Cobs(#[source] CobsError),
    #[error("Error during preamble: {0}")]
    Preamble(#[source] PreambleError),
    #[error("COBS frame longer than expected. Frame header:{0:?}, found byte: {1:02x}")]
    Overrun(FrameHeader, u8),
    #[error("Header cut off at position {1} after receiving: {0:?}")]
    HeaderCutoff([u8; 4], usize),
    #[error("Payload cut off at position {1} with frame header: {0:?}")]
    PayloadCutoff(FrameHeader, usize),
    #[error("CRC32 cut off at position {1} after receiving: {0:?}")]
    CrcCutoff([u8; 4], usize),
    #[error("Length byte at offset {0}: 0x{1:02x} should have highest two bits set")]
    LengthEncoding(usize, u8),
}
#[derive(Debug, Copy, Clone)]
enum FrameState {
    Preamble,
    Length(usize),
    PacketHeader(usize, usize),
    PacketBody(usize, FrameHeader),
    Legacy,
    LegacyPrintString(usize, usize),
}
#[derive(Debug)]
pub struct FrameLayer {
    preamble_layer: PreambleLayer,
    cobs_decoder: CobsDecoder,

    length_bytes: [u8; 4],
    header_bytes: [u8; 4],
    crc_bytes: [u8; 4],
    decode_state: FrameState,
    crc: crc32fast::Hasher,
}
#[derive(Debug, Copy, Clone)]
pub enum FrameOutput {
    Skip,
    Header(FrameHeader),
    Payload(u8),
    Finished,
    Legacy,
    LegacyPrintStringByte(usize, u8),
}
impl FrameLayer {
    pub fn new(cobs_xor: u8) -> Self {
        Self {
            preamble_layer: PreambleLayer::new(),
            cobs_decoder: CobsDecoder::new(cobs_xor),

            length_bytes: [0; 4],
            header_bytes: [0; 4],
            crc_bytes: [0; 4],
            decode_state: FrameState::Preamble,
            crc: crc32fast::Hasher::new(),
        }
    }
    pub fn skip_preamble(&mut self) {
        self.decode_state = FrameState::Length(0);
    }
    fn decode_header_bytes(&self, payload_len: usize) -> Result<FrameHeader, FrameError> {
        let message_type = u32::from_le_bytes(self.header_bytes[0..4].try_into().unwrap());
        let message_type = MessageType::try_from(message_type)
            .map_err(|_| FrameError::InvalidType(message_type))?;
        const _: () = assert!(size_of::<u32>() <= size_of::<usize>());
        Ok(FrameHeader {
            message_type,
            payload_len,
        })
    }
    pub fn reset(&mut self) {
        self.preamble_layer.reset();
        self.cobs_decoder.reset();

        self.length_bytes.fill(0);
        self.header_bytes.fill(0);
        self.crc_bytes.fill(0);
        self.decode_state = FrameState::Preamble;
        self.crc = crc32fast::Hasher::new();
    }
    pub fn feed(&mut self, byte: u8) -> Result<FrameOutput, FrameError> {
        match self.decode_state {
            FrameState::Legacy => Ok(FrameOutput::Legacy),
            FrameState::Preamble => {
                match self
                    .preamble_layer
                    .poll(byte)
                    .map_err(FrameError::Preamble)?
                {
                    PreambleStatus::Preamble => {
                        self.decode_state = FrameState::Length(0);
                        Ok(FrameOutput::Skip)
                    }
                    PreambleStatus::LegacyPutProgramInfo => {
                        self.decode_state = FrameState::Legacy;
                        Ok(FrameOutput::Legacy)
                    }
                    PreambleStatus::Skip => Ok(FrameOutput::Skip),
                    PreambleStatus::LegacyPutString(length) => {
                        self.decode_state = FrameState::LegacyPrintString(length as usize, 0);
                        Ok(FrameOutput::Skip)
                    }
                }
            }
            FrameState::Length(i) => {
                if (byte & 0xc0) != 0xc0 {
                    Err(FrameError::LengthEncoding(i, byte))
                } else {
                    self.length_bytes[i] = byte;
                    if i == 3 {
                        let sextets = self.length_bytes.map(|x| x & !0xc0);
                        let payload_len = sextets[0] as usize
                            | ((sextets[1] as usize) << 6)
                            | ((sextets[2] as usize) << 12)
                            | ((sextets[3] as usize) << 18);
                        self.decode_state = FrameState::PacketHeader(0, payload_len);
                    } else {
                        self.decode_state = FrameState::Length(i + 1);
                    }
                    Ok(FrameOutput::Skip)
                }
            }
            FrameState::PacketHeader(i, payload_len) => {
                match self.cobs_decoder.poll(byte).map_err(FrameError::Cobs)? {
                    CobsState::Skip => Ok(FrameOutput::Skip),
                    CobsState::Byte(byte) => {
                        self.decode_state = FrameState::PacketHeader(i + 1, payload_len);
                        self.header_bytes[i] = byte;
                        self.crc.write_u8(byte);
                        if i == 3 {
                            let frame_header = self.decode_header_bytes(payload_len)?;
                            self.decode_state = FrameState::PacketBody(i + 1, frame_header);
                            Ok(FrameOutput::Header(frame_header))
                        } else {
                            Ok(FrameOutput::Skip)
                        }
                    }
                    CobsState::Finished => Err(FrameError::HeaderCutoff(self.header_bytes, i)),
                }
            }
            FrameState::PacketBody(i, frame_header) => {
                match self.cobs_decoder.poll(byte).map_err(FrameError::Cobs)? {
                    CobsState::Skip => Ok(FrameOutput::Skip),
                    CobsState::Byte(byte) => {
                        self.decode_state = FrameState::PacketBody(i + 1, frame_header);
                        let packet_finishes_at_index = 4 + frame_header.payload_len;

                        if i < packet_finishes_at_index {
                            self.crc.write_u8(byte);
                            Ok(FrameOutput::Payload(byte))
                        } else if i < packet_finishes_at_index + 4 {
                            let offset = i - packet_finishes_at_index;
                            self.crc_bytes[offset] = byte;
                            Ok(FrameOutput::Skip)
                        } else {
                            Err(FrameError::Overrun(frame_header, byte))
                        }
                    }
                    CobsState::Finished => {
                        let crc =
                            core::mem::replace(&mut self.crc, crc32fast::Hasher::new()).finalize();
                        // TYPE + [payload] + CRC32
                        let expected_crc_begin = 4 + frame_header.payload_len;
                        let expected_cobs_payload_len = 4 + frame_header.payload_len + 4;
                        if i < expected_crc_begin {
                            Err(FrameError::PayloadCutoff(frame_header, i))
                        } else if i < expected_cobs_payload_len {
                            Err(FrameError::CrcCutoff(self.crc_bytes, i))
                        } else {
                            let frame_crc = u32::from_le_bytes(self.crc_bytes);
                            if frame_crc != crc {
                                Err(FrameError::InvalidCRC(crc, frame_crc))
                            } else {
                                self.decode_state = FrameState::Preamble;
                                Ok(FrameOutput::Finished)
                            }
                        }
                    }
                }
            }
            FrameState::LegacyPrintString(length, index) => {
                let next_index = index + 1;
                if next_index >= length {
                    self.decode_state = FrameState::Preamble;
                    Ok(FrameOutput::LegacyPrintStringByte(length, byte))
                } else {
                    self.decode_state = FrameState::LegacyPrintString(length, next_index);
                    Ok(FrameOutput::LegacyPrintStringByte(length, byte))
                }
            }
        }
    }
}
