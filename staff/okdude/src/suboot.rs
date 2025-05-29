use crate::tty::{ClearBuffer, Tty};
use crate::Args;
use color_eyre::{eyre, Section};
use eyre::bail;
use okboot_common::host::FormatDetails;
use okboot_common::su_boot::Command;
use std::io::{self, ErrorKind, Read, Write};
use std::process::exit;

struct Write32<'a> {
    inner: &'a mut Tty,
    quiet: bool,
}
impl<'a> Write32<'a> {
    pub fn new(inner: &'a mut Tty, quiet: bool) -> Self {
        Self { inner, quiet }
    }
    fn write32_le(&mut self, w: u32, ctrl: bool) -> io::Result<()> {
        if !self.quiet && ctrl {
            tracing::trace!("> writing {w:#010x}");
        }
        self.inner.write32_le(w)
    }
}

fn with_write32(
    tty: &mut Tty,
    quiet: bool,
    f: impl FnOnce(Write32) -> io::Result<()>,
) -> io::Result<()> {
    f(Write32::new(tty, quiet))
}

pub(crate) fn run(args: &Args, tty: &mut Tty) -> eyre::Result<()> {
    tracing::info!("[suboot]: Using legacy SU-BOOT protocol");

    tracing::debug!("[suboot] clearing buffers");
    tracing::warn!("[suboot] WARNING: any PRINT_STRINGs previously sent will be discarded");
    tty.clear(ClearBuffer::All)?;

    let FormatDetails::Bin { load_address } = args.format_details else {
        tracing::error!(
            "[suboot] legacy su-boot does not support {}",
            args.format_details
        );
        bail!("unsupported format");
    };

    let prog_data = std::fs::read(args.file.as_path())?;

    let crc32 = crc32fast::hash(&prog_data);

    // PUT_PROG_INFO

    tracing::debug!("[suboot] writing PUT_PROG_INFO");

    with_write32(tty, args.quiet, |mut w| {
        w.write32_le(Command::PutProgInfo as u32, true)?;
        w.write32_le(load_address.try_into().unwrap(), true)?;
        w.write32_le(prog_data.len().try_into().unwrap(), true)?;
        w.write32_le(crc32, true)?;
        Ok(())
    })?;

    tty.flush()?;

    let mut status = 0;
    let mut switch = 0;
    loop {
        let byte = tty.read8();
        if let Err(e) = byte {
            if e.kind() == ErrorKind::TimedOut {
                tracing::trace!("[suboot] failed to read from tty: {}", e);
            } else {
                tracing::debug!("[suboot] failed to read from tty: {}", e);
            }
            continue;
        }
        let byte = byte.unwrap();
        // if !args.quiet {
        //     tracing::trace!("< {byte}");
        // }
        // BOOT_ERROR (bbbbcccc) or GET_CODE (55556666)
        match (status, switch, byte) {
            (0, 0, 0xcc) => {
                switch = 1;
                status += 1
            }
            (1, 1, 0xcc) => status += 1,
            (2, 1, 0xbb) => status += 1,
            (3, 1, 0xbb) => status += 1,
            (0, 0, 0x66) => {
                switch = 2;
                status += 1
            }
            (1, 2, 0x66) => status += 1,
            (2, 2, 0x55) => status += 1,
            (3, 2, 0x55) => status += 1,
            (0, 0, 0xee) => {
                switch = 3;
                status += 1
            }
            (1, 3, 0xee) => status += 1,
            (2, 3, 0xdd) => status += 1,
            (3, 3, 0xdd) => {
                status = 0;
                switch = 0;
                let len = tty.read32_le().unwrap_or(0);
                if len > 0 {
                    let mut v = vec![0; len as usize];
                    let _ = tty.read_exact(&mut v);
                    tracing::info!("< {}", String::from_utf8_lossy(&v));
                }
            }
            (0, 0, _) => {}
            _ => {
                switch = 0;
                status = 0
            }
        }
        if status == 4 {
            break;
        }
    }

    if switch == 1 {
        // BOOT_ERROR
        tracing::error!(
            "[suboot] current settings would lead to a code collision on the device! Aborting."
        );
        exit(1);
    } else if switch == 2 {
        // GET_CODE
        tracing::debug!("[suboot] received GET_CODE");
    } else {
        unreachable!("state machine has two end states, 1 and 2; got neither");
    }

    let retransmitted_crc = tty
        .read32_le()
        .with_note(|| "while reading retransmitted CRC")?;

    if retransmitted_crc != crc32 {
        tracing::error!(
            "[suboot] bad CRC: sent {crc32:#010x}, received {retransmitted_crc:#010x}! Aborting."
        );
        exit(1);
    }
    tracing::info!("[suboot] received correct CRC, sending data");

    // PUT_CODE

    tracing::debug!("[suboot] writing PUT_CODE");
    with_write32(tty, args.quiet, |mut w| {
        w.write32_le(Command::PutCode as u32, true)
    })?;
    tracing::debug!("[suboot] writing data");
    tty.write_all(&prog_data)?;
    tracing::info!("[suboot] finished writing data");

    // wait for BOOT_START ?, BOOT_SUCCESS, BOOT_ERROR

    let mut status = 0;
    let mut switch = 0;
    loop {
        let byte = tty.read8();
        if let Err(e) = byte {
            if e.kind() == ErrorKind::TimedOut {
                tracing::trace!("failed to read from tty: {}", e);
            } else {
                tracing::debug!("failed to read from tty: {}", e);
            }
            continue;
        }
        let byte = byte.unwrap();
        // tracing::trace!("< {byte}");
        // BOOT_ERROR (bbbbcccc) or BOOT_SUCCESS (9999aaaa)
        match (status, switch, byte) {
            (0, 0, 0xcc) => {
                switch = 1;
                status += 1;
            }
            (1, 1, 0xcc) => status += 1,
            (2, 1, 0xbb) => status += 1,
            (3, 1, 0xbb) => status += 1,
            (0, 0, 0xaa) => {
                switch = 2;
                status += 1;
            }
            (1, 2, 0xaa) => status += 1,
            (2, 2, 0x99) => status += 1,
            (3, 2, 0x99) => status += 1,
            (0, 0, 0xee) => {
                switch = 3;
                status += 1;
            }
            (1, 3, 0xee) => status += 1,
            (2, 3, 0xdd) => status += 1,
            (3, 3, 0xdd) => {
                status = 0;
                switch = 0;
                let len = tty.read32_le().unwrap_or(0);
                if len > 0 {
                    let mut v = vec![0; len as usize];
                    let _ = tty.read_exact(&mut v);
                    tracing::info!("< {}", String::from_utf8_lossy(&v));
                }
            }
            (0, 0, _) => {}
            _ => {
                switch = 0;
                status = 0;
            }
        }
        if status == 4 {
            break;
        }
    }
    if switch == 1 {
        // BOOT_ERROR
        tracing::error!(
            "[suboot] current settings would lead to a code collision on the device! Aborting."
        );
        exit(1);
    } else if switch == 2 {
        // BOOT_SUCCESS
        tracing::info!("[suboot] device booted successfully.");
    } else {
        unreachable!("state machine has two end states, 1 and 2; got neither");
    }
    Ok(())
}
