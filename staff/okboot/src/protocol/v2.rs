use crate::buf::{FrameSink, SendError};
use crate::protocol::{ProtocolStatus, Timeouts};
use crate::rpc_println;
use crate::stub::flat_binary::{Integrity, Relocation};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt::Debug;
use core::time::Duration;
use elf::ElfBytes;
use elf::abi::{EM_ARM, ET_EXEC, PT_GNU_STACK, PT_LOAD, PT_NOTE, PT_TLS};
use elf::endian::{EndianParse, LittleEndian};
use elf::file::Class;
use elf::segment::Elf32_Phdr;
// use miniz_oxide::inflate::stream::InflateState;
// use miniz_oxide::{DataFormat, MZError, MZFlush, MZStatus};
use okboot_common::frame::FrameHeader;
use okboot_common::host::{Chunk, FormatDetails, Metadata};
use okboot_common::{MessageType, device, host};
use quartz::device::timing::{FloatingTimer, Instant};
use quartz::device::uart::Uart;
use thiserror::Error;

const CHUNK_SIZE: usize = 0x100;
mod timeouts {
    use crate::timeouts::RateRelativeTimeout;

    pub const TRY_RESEND: RateRelativeTimeout = RateRelativeTimeout::from_bytes(0x300);
    // 150% CHUNK_SIZE
    pub const TRY_RESEND_CHUNK: RateRelativeTimeout =
        RateRelativeTimeout::from_bytes(super::CHUNK_SIZE * 16);
    pub const BUFFER_RETRY: RateRelativeTimeout = RateRelativeTimeout::from_bytes(0x80);
}
#[derive(Debug, Copy, Clone)]
struct V1Timeouts {
    try_resend: Duration,
    buffer_retry: Duration,
    try_resend_chunk: Duration,
}

impl V1Timeouts {
    pub fn new_8n1(baud: u32) -> Self {
        Self {
            try_resend: timeouts::TRY_RESEND.at_baud_8n1(baud),
            buffer_retry: timeouts::BUFFER_RETRY.at_baud_8n1(baud),
            try_resend_chunk: timeouts::TRY_RESEND_CHUNK.at_baud_8n1(baud),
        }
    }
}

#[derive(Debug)]
enum S {
    /// expect: [`MetadataAck`], send: [`MetadataReq`]
    RequestMetadata,
    /// expect: [`MetadataAckAck`], send: [`MetadataAck`]
    AckMetadata(Metadata),
    /// expect: [`Chunk`], send: [`ChunkReq`]
    RequestChunk {
        which: usize,
        count: usize,
        loader: LoaderEnum,
    },
    /// expect: [`BootingAck`], send: [`Booting`]
    Boot { booter: Booter },
}

pub struct V2 {
    state: S,

    once: bool,
    retry_buffer: bool,
    // Tracks current retry count; if it reaches a certain number, an Abend will be forced
    retries: u8,
    heartbeat: Instant,

    baud: u32,
    timeouts: V1Timeouts,
    // inflate_state: InflateState,
    // remainder: usize,
}
impl Debug for V2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("V2")
            .field("state", &self.state)
            .field("once", &self.once)
            .field("retry_buffer", &self.retry_buffer)
            .field("retries", &self.retries)
            .field("heartbeat", &self.heartbeat)
            .field("baud", &self.baud)
            .field("timeouts", &self.timeouts)
            // .field("inflate_state", "<opaque>")
            .finish()
    }
}

impl V2 {
    pub fn new<FT: FloatingTimer>(ft: &FT, baud: u32) -> Self {
        Self {
            state: S::RequestMetadata,
            once: true,
            retry_buffer: false,
            retries: 0,
            heartbeat: Instant::now(ft),
            baud,
            timeouts: V1Timeouts::new_8n1(baud),
            // inflate_state: InflateState::new(DataFormat::Raw),
            // remainder: 0,
        }
    }
}

impl super::Protocol for V2 {
    fn handle_packet<U: Uart, FT: FloatingTimer>(
        &mut self,
        frame_header: FrameHeader,
        payload: &[u8],
        frame_sink: &mut FrameSink,
        timeouts: &mut Timeouts,
        uart: &U,
        _ft: &FT,
    ) -> ProtocolStatus {
        match frame_header.message_type {
            MessageType::Metadata => {
                rpc_println!(frame_sink, "[device/v2] V2Timeouts={:?}", self.timeouts);
                // rpc_println!(frame_sink, "[device/v2] received V2/Metadata");
                let msg: Metadata = match postcard::from_bytes(payload) {
                    Ok(msg) => msg,
                    Err(e) => {
                        rpc_println!(
                            frame_sink,
                            "[device/v2] failed to parse payload (V2/Metadata): {:?}",
                            e
                        );
                        return ProtocolStatus::Continue;
                    }
                };
                self.recv_metadata(msg, frame_sink, timeouts);
            }
            MessageType::MetadataAckAck => {
                // rpc_println!(frame_sink, "[device/v2] received V2/MetadataAckAck");
                let msg: host::MetadataAckAck = match postcard::from_bytes(payload) {
                    Ok(msg) => msg,
                    Err(e) => {
                        rpc_println!(
                            frame_sink,
                            "[device/v2] failed to parse payload (V2/MetadataAckAck): {:?}",
                            e
                        );
                        return ProtocolStatus::Continue;
                    }
                };
                self.recv_metadata_ack_ack(msg, frame_sink);
            }
            MessageType::Chunk => {
                // rpc_println!(frame_sink, "[device/v2] received V2/Chunk");
                let msg: Chunk = match postcard::from_bytes(payload) {
                    Ok(msg) => msg,
                    Err(e) => {
                        rpc_println!(
                            frame_sink,
                            "[device/v2] failed to parse payload (V2/Chunk): {:?}",
                            e
                        );
                        return ProtocolStatus::Continue;
                    }
                };
                if !self.recv_chunk(msg, frame_sink, uart) {
                    // if this returns fails, CRC failed or other catastrophic error
                    return ProtocolStatus::Abend;
                }
            }
            MessageType::BootingAck => {
                // rpc_println!(frame_sink, "[device/v2] received V2/BootingAck");
                self.recv_booting_ack(frame_sink, uart);
            }
            otherwise => {
                rpc_println!(
                    frame_sink,
                    "[device/v2] unrecognized message type: {:?}, ignoring",
                    otherwise
                );
            }
        }
        ProtocolStatus::Continue
    }

    fn heartbeat<FT: FloatingTimer>(
        &mut self,
        frame_sink: &mut FrameSink,
        _timeouts: &mut Timeouts,
        ft: &FT,
    ) -> ProtocolStatus {
        let send_once = core::mem::replace(&mut self.once, false);
        let heartbeat_elapsed = self.heartbeat.elapsed(ft);

        let should_send
            // A. first time message is being sent
            = send_once
            // B. retry due to no response - in the case of RequestChunk, we have a longer retry due
            //    to large message size
            || heartbeat_elapsed
            > if matches!(self.state, S::RequestChunk { .. }) {
            self.timeouts.try_resend_chunk
        } else {
            self.timeouts.try_resend
        }
            // C. retry due to failed send - retry after a while
            // XXX(mc): pretty sure this only happens if the buffer is full, so we're basically
            //          trying to make time to drain the buffer
            || (self.retry_buffer && heartbeat_elapsed > self.timeouts.buffer_retry);

        if should_send {
            if !send_once {
                self.retries += 1;
                if self.retries >= 10 {
                    return ProtocolStatus::Abend;
                }
            } else {
                self.retries = 0;
            }
            let send_result = match &self.state {
                S::RequestMetadata => self.send_metadata_request(frame_sink),
                S::AckMetadata(metadata) => self.send_metadata_ack(frame_sink, *metadata),
                S::RequestChunk {
                    which,
                    count: _,
                    loader: _,
                } => self.send_chunk_request(frame_sink, *which),
                S::Boot { .. } => self.send_boot_msg(frame_sink),
            };
            match send_result {
                Ok(true) => self.retry_buffer = false,
                Ok(false) => self.retry_buffer = true,
                Err(()) => return ProtocolStatus::Abend,
            }

            self.heartbeat = Instant::now(ft);
        }

        ProtocolStatus::Continue
    }
}

impl V2 {
    fn recv_metadata(
        &mut self,
        msg: host::Metadata,
        frame_sink: &mut FrameSink,
        timeouts: &mut Timeouts,
    ) {
        if !matches!(self.state, S::RequestMetadata) {
            rpc_println!(
                frame_sink,
                "[device/v2] received unexpected V2/Metadata in state: {:?}, ignoring.",
                self.state
            );
            return;
        }
        let ok = match msg.format_details {
            FormatDetails::Bin { load_address } => {
                if load_address < 0x2000_0000 || load_address >= 0x2008_2000 {
                    rpc_println!(
                        frame_sink,
                        "[device/v2] BIN file load address too high (must be in 0x2000_0000..0x2008_2000)"
                    );
                    false
                } else if (load_address & 3) != 0 {
                    rpc_println!(
                        frame_sink,
                        "[device/v2] BIN file load address must be 4-byte aligned"
                    );
                    false
                } else {
                    true
                }
            }
            FormatDetails::Elf => {
                rpc_println!(frame_sink, "[device/v2] Loading ELF file");
                true
            }
        };
        if ok {
            self.state = S::AckMetadata(msg);
            self.once = true;
            // override session timeout
            timeouts.override_session_timeout =
                Some(timeouts::TRY_RESEND_CHUNK.at_baud_8n1(self.baud) * 2)
        }
    }
    fn recv_metadata_ack_ack(&mut self, msg: host::MetadataAckAck, frame_sink: &mut FrameSink) {
        let S::AckMetadata(metadata) = &self.state else {
            rpc_println!(
                frame_sink,
                "[device/v2] received V2/MetadataAckAck in state: {:?}, ignoring.",
                self.state
            );
            return;
        };
        if !msg.is_ok {
            rpc_println!(
                frame_sink,
                "[device/v2] received V2/MetadataAckAck(ok=false), requesting metadata again"
            );
            self.state = S::RequestMetadata;
            return;
        }
        let chunk_count = (metadata.deflated_len as usize + CHUNK_SIZE - 1) / CHUNK_SIZE;
        let loader = match metadata.format_details {
            FormatDetails::Bin { load_address } => LoaderEnum::BinLoader(BinLoader::new(
                load_address.try_into().expect(
                    "cannot reach this point with load_address that is not representable as u32",
                ),
                metadata.clone(),
            )),
            FormatDetails::Elf => LoaderEnum::ElfLoader(ElfLoader::new(metadata.clone())),
        };
        self.state = S::RequestChunk {
            which: 0,
            count: chunk_count,
            loader,
        };
        self.once = true;
    }
    fn recv_chunk<U: Uart>(
        &mut self,
        msg: Chunk,
        frame_sink: &mut FrameSink,
        uart: &U,
        // _inflate_buffer: &mut [u8],
    ) -> bool {
        let S::RequestChunk {
            which,
            count,
            loader,
        } = &mut self.state
        else {
            rpc_println!(
                frame_sink,
                "[device/v2] received unexpected V2/Chunk(which={}) in state: {:?}, ignoring.",
                msg.which,
                self.state
            );
            return true;
        };
        let finished = if *which == msg.which as usize {
            // received correct chunk, pass it to the loader

            // let (a, b) = inflate_buffer.split_at_mut(0x2000);
            // {
            //     let new_end = msg.bytes.len() + self.remainder;
            //     assert!(a.len() >= new_end);
            //     a[self.remainder..new_end].copy_from_slice(msg.bytes);
            //     self.remainder = new_end;
            // }

            loop {
                // rpc_println!(frame_sink, "[device/v2] inflate on: {:02x?}", &a[..new_end]);
                // let inflate_result = miniz_oxide::inflate::stream::inflate(
                //     &mut self.inflate_state,
                //     &a[..self.remainder],
                //     b,
                //     MZFlush::None,
                // );
                // rpc_println!(
                //     frame_sink,
                //     "[device/v2] inflate to: {:02x?}",
                //     &b[..inflate_result.bytes_written]
                // );
                // rpc_println!(frame_sink, "[device/v2] inflate: {:?}", inflate_result);

                // let mut done = false;
                // match inflate_result.status {
                //     Ok(stat) => match stat {
                //         MZStatus::Ok => {
                //             a.copy_within(inflate_result.bytes_consumed..self.remainder, 0);
                //             self.remainder -= inflate_result.bytes_consumed;
                //             if inflate_result.bytes_written == 0 {
                //                 break;
                //             }
                //         }
                //         MZStatus::StreamEnd => {
                //             self.remainder -= inflate_result.bytes_consumed;
                //             done = true;
                //         }
                //         MZStatus::NeedDict => unreachable!(), // unused
                //     },
                //     Err(e) => match e {
                //         MZError::Buf => {
                //             // rpc_println!(frame_sink, "[device/v2] failed to make inflate progress");
                //             assert_eq!(inflate_result.bytes_consumed, 0);
                //             assert_eq!(inflate_result.bytes_written, 0);
                //             break;
                //         }
                //         MZError::Data => {
                //             rpc_println!(
                //                 frame_sink,
                //                 "[device/v2] MZError::Data probably indicates data corruption"
                //             );
                //             return false; // catastrophic
                //         }
                //         e => {
                //             rpc_println!(
                //                 frame_sink,
                //                 "[device/v2] unexpected error while inflating: {:?}",
                //                 e
                //             );
                //             return false; // catastrophic
                //         }
                //     },
                // }
                // if let Err(e) = loader.receive_bytes(&b[..inflate_result.bytes_written]) {
                //     rpc_println!(frame_sink, "[device/v2] unrecoverable load error: {}", e);
                //     return false; // catastrophic
                // }
                if let Err(e) = loader.receive_bytes(msg.bytes) {
                    rpc_println!(frame_sink, "[device/v2] unrecoverable load error: {}", e);
                    return false; // catastrophic
                }
                break;
                // if done {
                //     break;
                // }
            }

            *which += 1;
            if *which == *count {
                // done
                rpc_println!(frame_sink, "[device/v2] processed last chunk");
                true // finished
            } else {
                false // not finished
            }
        } else {
            rpc_println!(
                frame_sink,
                "[device/v2] wrong chunk, expected {} got {}",
                *which,
                msg.which
            );
            false // not finished
        };
        self.once = true;
        if finished {
            let S::RequestChunk {
                which: _,
                count: _,
                loader,
            } = core::mem::replace(&mut self.state, S::RequestMetadata)
            else {
                unreachable!()
            };
            let booter = match Loader::finalize(loader, frame_sink, uart) {
                Ok(booter) => booter,
                Err(e) => {
                    rpc_println!(frame_sink, "[device/v2] can't finalize, retrying: {e}");
                    return false;
                }
            };
            self.state = S::Boot { booter };
        }

        true
    }
    fn recv_booting_ack<U: Uart>(&mut self, frame_sink: &mut FrameSink, uart: &U) {
        if !matches!(self.state, S::Boot { .. }) {
            rpc_println!(
                frame_sink,
                "[device/v2] received unexpected V2/BootingAck in state {:?}, ignoring.",
                self.state
            );
            return;
        };
        rpc_println!(frame_sink, "[device/v2] received V2/BootingAck, booting");
        let S::Boot { booter } = core::mem::replace(&mut self.state, S::RequestMetadata) else {
            unreachable!()
        };
        booter.enter(uart, frame_sink)
    }

    fn send_metadata_request(&mut self, frame_sink: &mut FrameSink) -> Result<bool, ()> {
        match frame_sink.send(&device::MetadataReq {}) {
            Ok(()) => Ok(true),
            Err(SendError::Truncated) => Ok(false),
            Err(e) => {
                rpc_println!(
                    frame_sink,
                    "[device/v2] failed to send V2/MetadataReq: {}",
                    e
                );
                Err(())
            }
        }
    }
    fn send_metadata_ack(
        &mut self,
        frame_sink: &mut FrameSink,
        metadata: Metadata,
    ) -> Result<bool, ()> {
        match frame_sink.send(&device::MetadataAck {
            chunk_size: CHUNK_SIZE as u32,
            metadata,
        }) {
            Ok(()) => Ok(true),
            Err(SendError::Truncated) => Ok(false),
            Err(e) => {
                rpc_println!(
                    frame_sink,
                    "[device/v2] failed to send V2/MetadataAck: {}",
                    e
                );
                Err(())
            }
        }
    }
    fn send_chunk_request(&mut self, frame_sink: &mut FrameSink, which: usize) -> Result<bool, ()> {
        match frame_sink.send(&device::ChunkReq {
            which: which as u32,
        }) {
            Ok(()) => Ok(true),
            Err(SendError::Truncated) => Ok(false),
            Err(e) => {
                rpc_println!(frame_sink, "[device/v2] failed to send V2/ChunkReq: {}", e);
                Err(())
            }
        }
    }
    fn send_boot_msg(&mut self, frame_sink: &mut FrameSink) -> Result<bool, ()> {
        match frame_sink.send(&device::Booting {}) {
            Ok(()) => Ok(true),
            Err(SendError::Truncated) => Ok(false),
            Err(e) => {
                rpc_println!(frame_sink, "[device/v2] failed to send V2/Booting: {}", e);
                Err(())
            }
        }
    }
}

#[derive(Debug)]
enum Booter {
    Relocation {
        relocation: Relocation,
    },
    Elf {
        program_headers: Vec<Elf32_Phdr>,
        elf: Vec<u8>,
        entry: usize,
    },
}
impl Booter {
    fn flat_binary(relocation: Relocation) -> Self {
        Self::Relocation { relocation }
    }
    fn enter<U: Uart>(self, uart: &U, frame_sink: &mut FrameSink) -> ! {
        match self {
            Self::Relocation { relocation } => unsafe {
                crate::stub::flat_binary::final_relocation(frame_sink, relocation, uart)
            },
            Booter::Elf {
                program_headers,
                elf,
                entry,
            } => unsafe {
                crate::stub::elf::final_relocation(frame_sink, program_headers, &elf, entry)
            },
        }
    }
}

#[derive(Debug, Error)]
enum LoadError {
    #[error("CRC mismatch")]
    Crc,
    #[error("ELF error: {0}")]
    Elf(ElfError),
}

#[enum_dispatch::enum_dispatch]
#[derive(Debug)]
enum LoaderEnum {
    BinLoader,
    ElfLoader,
}

#[enum_dispatch::enum_dispatch(LoaderEnum)]
trait Loader: Debug {
    fn receive_bytes(&mut self, bytes: &[u8]) -> Result<(), LoadError>;
    fn finalize<U: Uart>(self, frame_sink: &mut FrameSink, uart: &U) -> Result<Booter, LoadError>;
}

#[derive(Debug)]
struct BinLoader {
    metadata: Metadata,

    relocation: Relocation,
    bytes_written: usize,
}
impl BinLoader {
    pub fn new(load_address: u32, metadata: Metadata) -> Self {
        let relocation = Relocation::calculate(
            load_address as usize,
            metadata.inflated_len as usize,
            unsafe { crate::stub::locate_end() }.addr(),
        );
        Self {
            metadata,
            relocation,
            bytes_written: 0,
        }
    }
}
impl Loader for BinLoader {
    fn receive_bytes(&mut self, bytes: &[u8]) -> Result<(), LoadError> {
        let ptr = unsafe {
            self.relocation
                .base_address_ptr
                .byte_offset(self.bytes_written as isize)
        };
        self.bytes_written += bytes.len();
        unsafe {
            self.relocation.write_bytes(ptr.cast::<u8>(), bytes);
        }
        Ok(())
    }

    fn finalize<U: Uart>(self, frame_sink: &mut FrameSink, uart: &U) -> Result<Booter, LoadError> {
        rpc_println!(frame_sink, "[device/v2] booter={self:?}");

        match unsafe {
            self.relocation.verify_integrity(
                self.metadata.inflated_crc,
                self.metadata.inflated_len as usize,
            )
        } {
            Integrity::Ok => {
                rpc_println!(frame_sink, "[device/v2] CRCs okay, running relocation stub");
                super::flush_to_fifo(frame_sink, uart);
                Ok(Booter::flat_binary(self.relocation.clone()))
            }
            Integrity::CrcMismatch {
                expected,
                calculated,
            } => {
                rpc_println!(
                    frame_sink,
                    "[device/v2] CRC mismatch: expected {:#010x} calculated {:#010x}",
                    expected,
                    calculated
                );
                super::flush_to_fifo(frame_sink, uart);
                Err(LoadError::Crc)
            }
        }
    }
}

#[derive(Debug)]
struct ElfLoader {
    #[allow(unused)]
    metadata: Metadata,
    bytes: Vec<u8>,
    program_headers: Vec<Elf32_Phdr>,
}
impl ElfLoader {
    pub fn new(metadata: Metadata) -> Self {
        Self {
            metadata,
            bytes: Vec::with_capacity(metadata.deflated_len as usize),
            program_headers: vec![],
        }
    }
}
#[derive(Debug, Error)]
pub enum ElfError {
    #[error("error parsing ELF header: {0}")]
    Parse(elf::ParseError),
    #[error("expected ELF32, found ELF64")]
    Class,
    #[error("expected EM_ARM")]
    Machine,
    #[error("expected ET_EXEC")]
    Type,
    #[error("expected ELF v1")]
    Version,
    #[error("expected entry in 0x2000_0000..0x2008_2000")]
    Entry,
    #[error("expected little-endian ELF binary")]
    Endianness,
    #[error("expected e_ident[EI_OSABI] to be 0 (none/sysv)")]
    OsAbi,
    #[error("expected a segment table")]
    NoSegmentTable,
    #[error("ELF TLS is not supported yet")]
    Tls,
    #[error("expected PT_LOAD or PT_TLS segment")]
    SegmentType,
    #[error("PT_LOAD must have p_filesz=p_memsz or p_filesz=0")]
    SegmentSize,
}
impl Loader for ElfLoader {
    fn receive_bytes(&mut self, bytes: &[u8]) -> Result<(), LoadError> {
        self.bytes.extend_from_slice(bytes);

        Ok(())
    }

    fn finalize<U: Uart>(
        mut self,
        frame_sink: &mut FrameSink,
        _uart: &U,
    ) -> Result<Booter, LoadError> {
        let calculated_crc = crc32fast::hash(&self.bytes);
        if calculated_crc != self.metadata.deflated_crc {
            rpc_println!(
                frame_sink,
                "[device/v2] CRC mismatch: expected {:#010x} calculated {:#010x}",
                self.metadata.deflated_crc,
                calculated_crc
            );
            return Err(LoadError::Crc);
        }
        let eb: ElfBytes<LittleEndian> = match elf::ElfBytes::minimal_parse(&self.bytes) {
            Ok(eb) => eb,
            Err(e) => {
                return Err(LoadError::Elf(ElfError::Parse(e)));
            }
        };
        let ehdr = &eb.ehdr;
        if !matches!(ehdr.class, Class::ELF32) {
            return Err(LoadError::Elf(ElfError::Class));
        }
        if !ehdr.endianness.is_little() {
            return Err(LoadError::Elf(ElfError::Endianness));
        }
        if ehdr.osabi != 0 {
            return Err(LoadError::Elf(ElfError::OsAbi));
        }

        if ehdr.e_type != ET_EXEC {
            return Err(LoadError::Elf(ElfError::Type));
        }
        if ehdr.e_machine != EM_ARM {
            return Err(LoadError::Elf(ElfError::Machine));
        }
        if ehdr.version != 1 {
            return Err(LoadError::Elf(ElfError::Version));
        }
        let entry = ehdr.e_entry;
        if entry < 0x2000_0000 || entry >= 0x2008_2000 {
            return Err(LoadError::Elf(ElfError::Entry));
        }

        let Some(segment_table) = eb.segments() else {
            return Err(LoadError::Elf(ElfError::NoSegmentTable));
        };
        for segment in segment_table.iter() {
            if segment.p_type == PT_TLS {
                return Err(LoadError::Elf(ElfError::Tls));
            } else if segment.p_type == PT_LOAD {
                // offset - offset in file
                // vaddr - "virtual" address to load at - we treat this as physical
                // paddr - physical address - ignored
                // filesz - size in file
                // memsz - size in memory
                // flags
                // align -  As ``Program Loading'' describes in this chapter of the processor
                //          supplement, loadable process segments must have congruent values for
                //          p_vaddr and p_offset, modulo the page size. This member gives the value
                //          to which the segments are aligned in memory and in the file. Values 0
                //          and 1 mean no alignment is required. Otherwise, p_align should be a
                //          positive, integral power of 2, and p_vaddr should equal p_offset, modulo
                //          p_align.
                if (segment.p_filesz != segment.p_memsz) && (segment.p_filesz != 0) {
                    return Err(LoadError::Elf(ElfError::SegmentSize));
                }
                self.program_headers.push(Elf32_Phdr {
                    p_type: segment.p_type,
                    p_offset: segment.p_offset as u32,
                    p_vaddr: segment.p_vaddr as u32,
                    p_paddr: segment.p_paddr as u32,
                    p_filesz: segment.p_filesz as u32,
                    p_memsz: segment.p_memsz as u32,
                    p_flags: segment.p_flags,
                    p_align: segment.p_align as u32,
                });
            } else if segment.p_type == PT_GNU_STACK {
                rpc_println!(frame_sink, "[device/v2/elf] ignoring PT_GNU_STACK");
            } else if segment.p_type == PT_NOTE {
                rpc_println!(frame_sink, "[device/v2/elf] ignoring PT_NOTE");
            } else {
                return Err(LoadError::Elf(ElfError::SegmentType));
            }
        }

        Ok(Booter::Elf {
            program_headers: self.program_headers,
            elf: self.bytes,
            entry: entry as usize,
        })
    }
}
