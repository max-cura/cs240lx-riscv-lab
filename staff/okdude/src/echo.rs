use crate::tty::Tty;
use crate::Args;
use color_eyre::eyre;
use std::io;
use std::io::{ErrorKind, Write};

pub fn echo(_args: &Args, tty: &mut Tty) -> eyre::Result<()> {
    // static ETERNAL_STDIN: OnceLock<Mutex<std::sync::mpsc::Receiver<String>>> = OnceLock::new();
    // let rx = ETERNAL_STDIN.get_or_init(|| {
    //     let (tx, rx) = std::sync::mpsc::channel();
    //     std::thread::spawn(move || {
    //         for line in io::stdin().lines() {
    //             if line.is_err() {
    //                 tracing::error!("[echo]: failed to read from stdin: {}", line.unwrap_err());
    //             } else {
    //                 let line = line.unwrap();
    //                 tx.send(line).unwrap();
    //             }
    //         }
    //     });
    //     Mutex::new(rx)
    // });
    loop {
        match tty.read8() {
            Ok(b) => {
                // if b.is_ascii_graphic() || b == b' ' || b == b'\n' || b == b'\r' || b == b'\t' {
                io::stdout().write_all(&[b])?
                // } else {
                // eprint!("<{:02x}>", b);
                // }
            }
            Err(e) => {
                if e.kind() != ErrorKind::TimedOut {
                    tracing::error!(
                        "[echo]: error on {}: {}",
                        tty.path()
                            .file_name()
                            .unwrap_or(tty.path().as_os_str())
                            .to_string_lossy(),
                        e
                    );
                    std::process::exit(1);
                }
            }
        }
        // match rx.lock().unwrap().try_recv() {
        //     Ok(s) => match tty.write_all(s.as_bytes()) {
        //         Ok(_) => {
        //             tracing::trace!("> wrote successfully")
        //         }
        //         Err(e) => {
        //             tracing::error!("[echo]: failed to send: {}", e)
        //         }
        //     },
        //     Err(TryRecvError::Empty) => {}
        //     Err(TryRecvError::Disconnected) => {
        //         tracing::error!("[echo]: ETERNAL_STDIN disconnected");
        //     }
        // }
    }
}
