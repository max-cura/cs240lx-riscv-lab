use crate::tty::Tty;
use crate::Args;
use elf::endian::LittleEndian;
use elf::ElfBytes;
use eyre::{bail, ensure, eyre, Result, WrapErr};
use indicatif::{ProgressBar, ProgressStyle};
use okboot_common::frame::{FrameHeader, FrameLayer, FrameOutput};
use okboot_common::host::FormatDetails;
use okboot_common::{device, host, EncodeMessageType, MessageType, COBS_XOR, INITIAL_BAUD_RATE};
use serde::Serialize;
use std::fmt::Debug;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::sync::{mpsc, Arc};

pub struct Decoder {
    received_messages: Sender<(MessageType, Vec<u8>)>,

    decoder: FrameLayer,
    frame_header: Option<FrameHeader>,
    buffer: Vec<u8>,
}

impl Decoder {
    fn reset(&mut self) {
        self.decoder.reset();
        self.frame_header = None;
        self.buffer.clear();
    }

    pub fn process_incoming_bytes(&mut self, buf: &[u8]) -> Result<()> {
        for &byte in buf {
            match self.decoder.feed(byte) {
                Ok(frame_output) => match frame_output {
                    FrameOutput::Skip => {}
                    FrameOutput::Header(frame_header) => {
                        if self.frame_header.is_some() {
                            bail!("received second frame header");
                        }
                        self.frame_header = Some(frame_header);
                        self.buffer.clear();
                    }
                    FrameOutput::Payload(byte) => {
                        ensure!(
                            self.frame_header.is_some(),
                            "received payload byte when no frame header has been received"
                        );
                        self.buffer.push(byte);
                    }
                    FrameOutput::Finished => {
                        let Some(frame_header) = self.frame_header.take() else {
                            bail!(
                                "received payload finished when no frame header has been received"
                            );
                        };
                        if frame_header.message_type == MessageType::PrintString {
                            tracing::info!(
                                "< {}",
                                std::str::from_utf8(&self.buffer)
                                    .unwrap_or("<invalid UTF-8>")
                                    .trim_end()
                            );
                        } else {
                            self.received_messages.send((
                                frame_header.message_type,
                                std::mem::take(&mut self.buffer),
                            ))?;
                        }
                    }
                    FrameOutput::Legacy => {
                        bail!("receiving SUBOOT PUT_PROG_INFO frame");
                    }
                    FrameOutput::LegacyPrintStringByte(length, byte) => {
                        self.buffer.push(byte);
                        if length == self.buffer.len() {
                            tracing::info!(
                                "< {}",
                                String::from_utf8_lossy(&self.buffer).trim_end()
                            );
                            // in theory, only need self.buffer.clear() here
                            self.reset();
                        }
                    }
                },
                Err(e) => {
                    self.reset();
                    return Err(e.into());
                }
            }
        }
        Ok(())
    }
}

pub fn drive(
    outgoing_messages: Receiver<Vec<u8>>,
    mut decoder: Decoder,
    tty: &mut Tty,
    close: Arc<AtomicBool>,
) -> Result<()> {
    'drive: loop {
        if close.load(Ordering::SeqCst) {
            break 'drive Ok(());
        }

        'push: loop {
            match outgoing_messages.try_recv() {
                Ok(m) => {
                    if let Err(e) = tty.write_all(m.as_slice()) {
                        tracing::error!("[v2] failed to write queued message: {e}");
                    }
                    if let Err(e) = tty.flush() {
                        tracing::error!("[v2] failed to flush message: {e}");
                    }
                }
                Err(TryRecvError::Empty) => break 'push,
                Err(TryRecvError::Disconnected) => break 'drive Ok(()),
            }
        }

        if let Ok(can_read_n) = tty.bytes_to_read() {
            if can_read_n > 0 {
                let mut v = vec![0; can_read_n];
                let buf = match tty.read(&mut v) {
                    Ok(buf_len) => &v[..buf_len],
                    Err(e) => {
                        tracing::error!("[v2] failed to read from tty: {e}");
                        continue 'drive;
                    }
                };

                if let Err(e) = decoder.process_incoming_bytes(buf) {
                    tracing::error!("[v2] failed to process incoming bytes: {e}");
                    continue 'drive;
                }
            }
        }
    }
}

struct Info {
    pub compressed_len: u32,
    pub decompressed_len: u32,

    pub compressed_crc: u32,
    pub decompressed_crc: u32,

    pub chunk_size: usize,
    // pub num_compressed_chunks: usize,
}

type Tx = Sender<Vec<u8>>;
fn send<M: EncodeMessageType + Serialize + Debug>(msg: &M, tx: &mut Tx) -> Result<()> {
    let wire_bytes = super::upload::encode(msg)?;
    tx.send(wire_bytes)?;
    Ok(())
}

fn serialize_args(args: &Args) -> Vec<u8> {
    let v: Vec<Vec<u8>> = args.args.iter().map(|x| x.clone().into_bytes()).collect();
    let mut v_out = vec![];
    v_out.extend_from_slice(&u32::to_le_bytes(v.len() as u32));
    for sub_v in &v {
        v_out.extend_from_slice(&u32::to_le_bytes(sub_v.len() as u32));
        v_out.extend_from_slice(&sub_v);
        v_out.extend(std::iter::repeat_n(0u8, 4 - (sub_v.len() % 4)));
        assert!(v_out.len().is_multiple_of(4));
    }
    v_out
}

fn upload_inner(
    args: &Args,
    mut out_tx: Tx,
    in_rx: Receiver<(MessageType, Vec<u8>)>,
) -> Result<()> {
    let mut uncompressed = std::fs::read(&args.file)
        .with_context(|| eyre!("failed to open {}", args.file.display()))?;

    if matches!(args.format_details, FormatDetails::Elf) {
        let arg_vector = serialize_args(args);
        let elf = ElfBytes::<LittleEndian>::minimal_parse(&uncompressed)
            .expect("failed to parse input ELF file");
        let stack = elf
            .section_header_by_name(".data.args")
            .expect("section table should be parseable");
        if let Some(stack) = stack {
            let offset = stack.sh_offset as usize;
            let size = stack.sh_size as usize;
            if arg_vector.len() >= size {
                tracing::error!("arguments would occupy more space than .data.args section");
                std::process::exit(1);
            }
            let _ = elf;
            tracing::debug!("Found .data.args : offset={offset} size={size}");
            uncompressed[offset..offset + arg_vector.len()].copy_from_slice(&arg_vector);
            tracing::info!("Inserted arguments in .data.args");
        }
    }

    // let compressed = miniz_oxide::deflate::compress_to_vec(&uncompressed, 5);
    // tracing::info!("[v2] compressed: {compressed:x?}");
    tracing::info!("[v2] original file length: {}", uncompressed.len());
    // tracing::info!("[v2] compressed file length: {}", compressed.len());
    let crc = crc32fast::hash(&uncompressed);
    // let crc_compressed = crc32fast::hash(&compressed);
    let mut info = Info {
        compressed_len: uncompressed.len() as u32, //compressed.len() as u32,
        decompressed_len: uncompressed.len() as u32,

        compressed_crc: crc, //crc_compressed,
        decompressed_crc: crc,

        chunk_size: 0,
        // num_compressed_chunks: 0,
    };
    let mut progress_bar = ProgressBar::new_spinner();

    tracing::info!("[v2] waiting for device to commence upload process");

    loop {
        match in_rx.try_recv() {
            Ok((typ, msg)) => match typ {
                MessageType::AllowedVersions => {
                    tracing::warn!("[v2] ignoring leftover Handshake/AllowedVersions");
                }
                MessageType::MetadataReq => {
                    let msg: device::MetadataReq = match postcard::from_bytes(&msg) {
                        Ok(x) => x,
                        Err(e) => {
                            tracing::error!(
                                "[v2] failed to deserialize incoming message (MetadataReq): {e}"
                            );
                            continue;
                        }
                    };
                    dispatch_metadata_req(msg, &info, &args.format_details, &mut out_tx);
                }
                MessageType::MetadataAck => {
                    let msg: device::MetadataAck = match postcard::from_bytes(&msg) {
                        Ok(x) => x,
                        Err(e) => {
                            tracing::error!(
                                "[v2] failed to deserialize incoming message (MetadataAck) from bytes {msg:?}: {e}"
                            );
                            continue;
                        }
                    };
                    match dispatch_metadata_ack(msg, &info, &args.format_details, &mut out_tx) {
                        Ok((new_info, new_pb)) => {
                            info = new_info;
                            progress_bar = new_pb;
                        }
                        Err(e) => {
                            tracing::error!("[v2] problem with metadata ack: {e}");
                        }
                    }
                }
                MessageType::ChunkReq => {
                    let msg: device::ChunkReq = match postcard::from_bytes(&msg) {
                        Ok(x) => x,
                        Err(e) => {
                            tracing::error!(
                                "[v2] failed to deserialize incoming message (ChunkReq): {e}"
                            );
                            continue;
                        }
                    };
                    dispatch_chunk_req(msg, &info, &uncompressed, &mut out_tx, &progress_bar);
                }
                MessageType::Booting => {
                    let out_msg = host::BootingAck {};
                    if let Err(e) = send(&out_msg, &mut out_tx) {
                        tracing::error!("[v2] failed to send {msg:?}: {e}, continuing.");
                    }
                    tracing::info!("[v2] device is booting");
                    break;
                }
                t => {
                    tracing::error!("[v2] unrecognized message type: {t:?}, ignoring.");
                }
            },
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => {
                tracing::error!("[v2] device disconnected");
                bail!("internal message queue disconnected");
            }
        }
    }
    Ok(())
}

fn dispatch_metadata_req(
    _msg: device::MetadataReq,
    info: &Info,
    format_details: &FormatDetails,
    tx: &mut Tx,
) {
    tracing::info!("[v2] received V2/MetadataReq");
    let msg = host::Metadata {
        deflated_crc: info.compressed_crc,
        deflated_len: info.compressed_len,
        inflated_crc: info.decompressed_crc,
        inflated_len: info.decompressed_len,
        format_details: format_details.clone(),
    };
    if let Err(e) = send(&msg, tx) {
        tracing::error!("[v2] failed to send {msg:?}: {e}, continuing.");
    }
}
fn dispatch_metadata_ack(
    msg: device::MetadataAck,
    info: &Info,
    expected_format_details: &FormatDetails,
    tx: &mut Tx,
) -> Result<(Info, ProgressBar)> {
    tracing::info!("[v2] received V2/MetadataAck");
    let host::Metadata {
        deflated_crc,
        deflated_len,
        inflated_crc,
        inflated_len,
        format_details,
    } = msg.metadata.clone();
    let deflated_crc_ok = deflated_crc == info.compressed_crc;
    let deflated_len_ok = deflated_len == info.compressed_len;
    let inflated_crc_ok = inflated_crc == info.decompressed_crc;
    let inflated_len_ok = inflated_len == info.decompressed_len;
    let format_details_ok = &format_details == expected_format_details;
    if !deflated_crc_ok {
        tracing::error!(
            "[v2] compressed CRC mismatch: expected {:08x} received {:08x}",
            info.compressed_crc,
            deflated_crc
        );
    }
    if !deflated_len_ok {
        tracing::error!(
            "[v2] compressed length mismatch: expected {} received {}",
            info.compressed_len,
            deflated_len
        );
    }
    if !inflated_crc_ok {
        tracing::error!(
            "[v2] decompressed CRC mismatch: expected {:08x} received {:08x}",
            info.decompressed_crc,
            inflated_crc
        );
    }
    if !inflated_len_ok {
        tracing::error!(
            "[v2] decompressed length mismatch: expected {:08x} received {:08x}",
            info.decompressed_len,
            inflated_len
        );
    }
    if !format_details_ok {
        tracing::error!(
            "[v2] format details mismatch: expected {expected_format_details:?} received {format_details:?}"
        );
    }
    let ok = deflated_crc_ok
        && deflated_len_ok
        && inflated_crc_ok
        && inflated_len_ok
        && format_details_ok;
    let out_msg = &host::MetadataAckAck { is_ok: ok };
    if let Err(e) = send(out_msg, tx) {
        tracing::error!("[v2] failed to send {out_msg:?}: {e}, continuing.");
        return Err(e);
    }
    if ok {
        // let num_compressed_chunks = (info.compressed_len as usize + msg.chunk_size as usize - 1)
        //     / (msg.chunk_size as usize);
        let pb = ProgressBar::new(info.compressed_len as u64);
        pb.set_style(ProgressStyle::with_template(
            "[{elapsed_precise}] {bar:60.cyan/blue} [{bytes:}/{total_bytes}] {bytes_per_sec}",
        )?);

        Ok((
            Info {
                chunk_size: msg.chunk_size as usize,
                // num_compressed_chunks,
                ..*info
            },
            pb,
        ))
    } else {
        bail!("incorrect metadata ack")
    }
}

fn dispatch_chunk_req(
    msg: device::ChunkReq,
    info: &Info,
    compressed_data: &[u8],
    tx: &mut Tx,
    progress_bar: &ProgressBar,
) {
    tracing::trace!("[v2] received V2/ChunkReq(which={})", msg.which);
    let chunk_idx = msg.which as usize;
    let chunk_begin = chunk_idx * info.chunk_size;
    let chunk_end = (chunk_begin + info.chunk_size).min(compressed_data.len());

    progress_bar.update(|s| s.set_pos(chunk_begin as u64));

    let out_msg = &host::Chunk {
        which: msg.which,
        bytes: &compressed_data[chunk_begin..chunk_end],
    };
    if let Err(e) = send(out_msg, tx) {
        tracing::error!("[v2] failed to send {msg:?}: {e}, continuing.");
    }
}

pub fn upload(args: &Args, tty: &mut Tty) -> Result<()> {
    let close = Arc::new(AtomicBool::new(false));
    let (out_tx, out_rx) = mpsc::channel();
    let (in_tx, in_rx) = mpsc::channel();
    let succeeded = std::thread::scope(|scope| {
        let close2 = Arc::clone(&close);
        let jh = scope.spawn(|| {
            drive(
                out_rx,
                Decoder {
                    received_messages: in_tx,
                    decoder: FrameLayer::new(COBS_XOR),
                    frame_header: None,
                    buffer: vec![],
                },
                tty,
                close2,
            )
        });

        let r = match upload_inner(args, out_tx, in_rx) {
            Ok(_) => true,
            Err(e) => {
                tracing::error!("[v2] upload failed: {e}");
                false
            }
        };

        close.store(true, Ordering::SeqCst);
        if let Err(e) = jh.join().unwrap() {
            tracing::error!("[v2] driver thread error: {e}");
        }
        r
    });
    tty.set_baud_rate(INITIAL_BAUD_RATE)?;
    tracing::info!("[v2] switching to echo mode");
    if !succeeded {
        tracing::error!("[v2] aborting");
        std::process::exit(1);
    }
    Ok(())
}
