use color_eyre::{eyre, Result};
use nix::poll::{PollFd, PollFlags, PollTimeout};
use nix::{ioctl_read, ioctl_write_ptr_bad};
use std::ffi::{c_int, CString};
use std::io::{ErrorKind, Read, Write};
use std::mem::MaybeUninit;
use std::os::fd::BorrowedFd;
use std::path::{Path, PathBuf};
use std::process::exit;
use std::time::Duration;
use std::{io, ptr, slice};
use tracing::instrument;

#[derive(Debug, Copy, Clone)]
pub enum ClearBuffer {
    #[allow(unused)]
    Input,
    #[allow(unused)]
    Output,
    All,
}

pub struct Tty {
    fd: c_int,
    default_timeout: Duration,
    path: PathBuf,
}

const IOSSIOSPEED: libc::c_ulong = 0x80045402;
ioctl_write_ptr_bad!(
    #[cfg(any(target_os = "ios", target_os = "macos"))]
    iossiospeed,
    IOSSIOSPEED,
    libc::speed_t
);

ioctl_read!(fionread, b'f', 127, libc::c_int);

impl Tty {
    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn read_timeout(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        if buf.len() == 0 {
            return Ok(0);
        }

        self._read_timeout(buf, timeout)
    }

    #[allow(dead_code)]
    pub fn read_nonblocking(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.len() == 0 {
            return Ok(0);
        }

        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n == -1 {
            let errno = nix::errno::Errno::last_raw();
            if errno == libc::EAGAIN {
                Err(io::Error::new(
                    io::ErrorKind::WouldBlock,
                    "no available data on TTY",
                ))
            } else {
                Err(nix::errno::Errno::last().into())
            }
        } else {
            Ok(n as usize)
        }
    }
}

impl Read for Tty {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        // Invariants:
        //  Put bytes in buffer, returning how many were read
        //  no guarantees about whether it blocks, but if an object needs to block for a read and
        //  cannot, signal via an Err value
        //  Ok(n) --> 0 <= n <= buf.len()
        //  nonzero n indicates that the buffer buf has been filled in with n bytes of data
        //  n=0 indicates that:
        //   A. reader has reached EOF and will likely no longer be able to produce bytes; note that
        //      this does not mean that the reader will *always* no longer be able to produce bytes
        //   B. buffer specified was 0 bytes in length
        if buf.len() == 0 {
            return Ok(0);
        }

        // We implement this as: timeout read!
        self.read_timeout(buf, self.default_timeout)
    }
}

impl Write for Tty {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        // log::trace!("poll start");
        self._can_write()?;

        // log::trace!("poll finished");

        // Invariants:
        //  Attempt to write entire contents
        //  A single attempt to write to wrapped object
        //  Not guaranteed to block waiting for data to be written
        //  A write which otherwise would block can be indicated through an Err variant
        //  If method consumed n>0 bytes of buf, it must return Ok(n)
        //  If the return value is Ok(n), then n must satisfy n<=buf.len()
        //  A return value of Ok(0) typically means that the underlying object is no longer able to
        //  accept bytes and will likely not be able to in the future as well, or that the buffer
        //  provided is empty.
        let n = unsafe {
            // write attempts to write nbyte of data to the object referenced by the descriptor from
            // the specified buffer.
            // when using non-blocking I/O on objects that are subject to flow control, write() may
            // write fewer bytes than requested; the return value must be noted, and the remainder
            // of the operation should be retried when possible.

            // problem : we want to write when
            libc::write(self.fd, buf.as_ptr() as *const libc::c_void, buf.len())
        };
        if n == -1 {
            // error
            if nix::errno::Errno::last_raw() == libc::EAGAIN {
                Err(std::io::Error::new(
                    std::io::ErrorKind::WouldBlock,
                    "write() would block",
                ))
            } else {
                Err(nix::errno::Errno::last().into())
            }
        } else {
            if n == 0 {
                Err(std::io::Error::new(
                    std::io::ErrorKind::WouldBlock,
                    "write() returned 0",
                ))
            } else {
                Ok(n as usize)
            }
        }
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let r = unsafe { libc::tcdrain(self.fd) };
        if r == -1 {
            Err(nix::errno::Errno::last().into())
        } else {
            Ok(())
        }
    }
}

impl Tty {
    #[instrument(skip(path), fields(path = ?path.as_ref()))]
    pub fn new<P: AsRef<Path>>(path: P, baud: u32) -> Result<Self> {
        let path_cstr = CString::new(path.as_ref().as_os_str().as_encoded_bytes())?;
        unsafe {
            // tracing::trace!("calling libc::open");
            let fd = libc::open(
                path_cstr.into_raw(),
                // readwrite, NOCTTY, SYNC, close-on-exec, don't block on open or for data to become
                // available
                libc::O_RDWR | libc::O_NOCTTY | libc::O_SYNC | libc::O_CLOEXEC | libc::O_NONBLOCK,
            );
            // tracing::trace!("libc::open returned {fd}");
            if fd >= 0 {
                let mut this = Self {
                    fd,
                    default_timeout: Duration::new(0, 0),
                    path: path.as_ref().to_path_buf(),
                };
                this.set_baud_rate(baud)?;
                // tracing::trace!("finished setting baud rate");
                Ok(this)
            } else {
                eyre::bail!(
                    "failed to open file: {}: error={fd}",
                    path.as_ref().display()
                );
            }
        }
    }

    pub fn set_baud_rate(&mut self, baud: u32) -> Result<()> {
        unsafe { self._set_speed(baud, true) }
    }
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.default_timeout = timeout;
        Ok(())
    }
    pub fn clear(&mut self, cb: ClearBuffer) -> Result<()> {
        let r = match cb {
            ClearBuffer::Input => unsafe { libc::tcflush(self.fd, libc::TCIFLUSH) },
            ClearBuffer::Output => unsafe { libc::tcflush(self.fd, libc::TCOFLUSH) },
            ClearBuffer::All => unsafe { libc::tcflush(self.fd, libc::TCIOFLUSH) },
        };
        if r == -1 {
            Err(nix::errno::Errno::last().into())
        } else {
            Ok(())
        }
    }

    pub fn bytes_to_read(&self) -> Result<usize> {
        let mut data = MaybeUninit::uninit();
        let bytes = {
            let res = unsafe { fionread(self.fd, data.as_mut_ptr())? };
            if res == -1 {
                return Err(nix::errno::Errno::last().into());
            } else {
                unsafe { data.assume_init() }
            }
        };
        Ok(bytes as usize)
    }

    /// Returns Ok(n>0) if bytes were read
    fn _read_timeout(&mut self, buf: &mut [u8], t: Duration) -> std::io::Result<usize> {
        if !self._can_read_timeout(t)? {
            Err(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "_read_timeout timed out",
            ))
        } else {
            let n =
                unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
            if n == -1 {
                Err(nix::errno::Errno::last().into())
            } else {
                // log::trace!("read {n}: {}", hexify(&buf[..n as usize]));
                Ok(n as usize)
            }
        }
    }

    fn _can_write(&mut self) -> io::Result<()> {
        use nix::errno::Errno::{EIO, EPIPE};

        let mut fd = PollFd::new(
            unsafe { BorrowedFd::borrow_raw(self.fd) },
            PollFlags::POLLOUT,
        );

        // let milliseconds = timeout.as_secs() as i64 * 1000 + i64::from(timeout.subsec_nanos()) / 1_000_000;
        let wait_res = nix::poll::poll(
            slice::from_mut(&mut fd),
            // select takes timeout 0 for indefinite block
            // poll takes timeout -1 for indefinite block
            /*milliseconds as nix::libc::c_int*/
            /* -1 */
            PollTimeout::NONE,
        );

        let wait = match wait_res {
            Ok(r) => r,
            Err(e) => return Err(e.into()),
        };
        // log::trace!("poll returned {wait}");
        // All errors generated by poll or ppoll are already caught by the nix wrapper around libc, so
        // here we only need to check if there's at least 1 event
        if wait != 1 {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "Operation timed out",
            ));
        }

        // Check the result of ppoll() by looking at the revents field
        match fd.revents() {
            Some(e) if e == PollFlags::POLLOUT => return Ok(()),
            // If there was a hangout or invalid request
            Some(e) if e.contains(PollFlags::POLLHUP) || e.contains(PollFlags::POLLNVAL) => {
                return Err(io::Error::new(io::ErrorKind::BrokenPipe, EPIPE.desc()));
            }
            Some(_) | None => (),
        }

        Err(io::Error::new(io::ErrorKind::Other, EIO.desc()))
    }

    fn _can_read_timeout(&mut self, t: Duration) -> std::io::Result<bool> {
        let mut rfds = {
            let mut rfds_fake = MaybeUninit::uninit();
            unsafe {
                libc::FD_ZERO(rfds_fake.as_mut_ptr());
                libc::FD_SET(self.fd, rfds_fake.as_mut_ptr());
            }
            unsafe { rfds_fake.assume_init() }
        };
        let mut timeval = libc::timeval {
            tv_sec: t.as_secs().try_into().map_err(|_| {
                std::io::Error::new(std::io::ErrorKind::Unsupported, "invalid timeout: seconds")
            })?, // c_long
            tv_usec: t.subsec_micros().try_into().map_err(|_| {
                std::io::Error::new(
                    std::io::ErrorKind::Unsupported,
                    "invalid timeout: microseconds",
                )
            })?, // i32
        };
        let r = unsafe {
            libc::select(
                self.fd + 1, // weird but that's how select works
                ptr::addr_of_mut!(rfds),
                ptr::null_mut(),
                ptr::null_mut(),
                ptr::addr_of_mut!(timeval),
            )
        };
        if r < 0 {
            Err(nix::errno::Errno::last().into())
        } else {
            let crt = unsafe { libc::FD_ISSET(self.fd, std::ptr::addr_of!(rfds)) };
            // log::trace!("crt={crt}");
            Ok(crt)
        }
    }

    #[instrument(skip(self))]
    unsafe fn _set_speed(&mut self, baud: u32, drain: bool) -> Result<()> {
        let mut tios = {
            let mut tios_fake: MaybeUninit<libc::termios> = MaybeUninit::uninit();
            // SAFETY: safe as long as `tcgetattr()` sets the `termios` information correctly.
            let r = unsafe { libc::tcgetattr(self.fd, tios_fake.as_mut_ptr()) };
            if r != 0 {
                eyre::bail!("failed to tcgetattr: error={r}");
            }
            // SAFETY: tcgetattr() expects an uninitialized pointer to a `termios` struct, so we
            // give it one.
            unsafe { tios_fake.assume_init() }
        };

        // ignore speed field: no cfsetspeed

        // println!("{tios:?}");

        // Noncanonical mode:
        //  input bytes are not assembled into lines, and erase and kill processing does not occur.
        // Writing data and output processing:
        //  when a process writes one or more bytes to a terminal device file, they are processed
        //  according to the c_oflag field. The implementation may provide a buffering mechanism; as
        //  such, when a call to write() completes, all of the bytes written have been scheduled for
        //  transmission to the device, but the transmission will not necessarily have been
        //  completed.
        // Special characters:
        //  INTR    if ISIG enabled, generates SIGINT. (disable)
        //  QUIT    if ISIG enabled, generates SIGQUIT. (disable)
        //  ERASE   if ICANON set, erases last character in current line. (disable)
        //  KILL    if ICANON set, deletes the entire line. (disable)
        //  EOF     if ICANON set, (disable)
        //  CR      if ICANON set
        //  NL      if ICANON set
        //  EOL     if ICANON set
        //  SUSP    if ISIG enabled
        //  STOP    if IXON or IXOFF is set
        //  START   if IXON or IXOFF is set
        //  EOL2        same as EOL
        //  WERASE  if ICANON
        //  REPRINT if ICANON
        //  DSUSP       similar SUSP
        //  LNEXT   if IEXTEN set ; receipt of this character causes the next character to be taken
        //          literally
        //  DISCARD if IEXTEN set ; recept of this character toggles the flushing of terminal output
        //  STATUS  if ICANON
        // General Terminal Interface:
        //  Last process to close a terminal device file causes any output to be sent to the device
        //  and any input to be discarded.

        // INPUT MODES:
        //  c_iflag
        //      IGNBRK  = ignore BREAK condition
        //      BRKINT  = map BREAK to SIGINTR
        //      IGNPAR  = discard parity errors
        //      PARMRK  = mark parity and framing errors
        //      INPCK   = enable checking of parity errors
        //      ISTRIP  = strip 8th bit off chars
        //      INLCR   = map NL into CR
        //      IGNCR   = ignore CR
        //      ICRNL   = map CR to NL (aka CRMOD)
        //      IXON    = enable output flow control
        //      IXOFF   = enable input flow control
        //      IXANY   = any char will restart after stop
        //      IMAXBEL = ring bell on input queue full
        //      IUCLC   = translate upper case to lower case
        // ignore breaks (staff code had this wrong)
        tios.c_iflag |= libc::IGNBRK;
        // Disable XON/XOFF flow control in both directions
        tios.c_iflag &= !(libc::IXON | libc::IXOFF | libc::IXANY);
        // Noncanonical mode
        tios.c_iflag &= !(libc::ICANON | libc::ECHO | libc::ECHOE | libc::ISIG);

        // OUTPUT MODES:
        //  c_oflag
        //      OPOST   = enable following ouptut processing
        //      ONLCR   = map NL to CR-NL (aka CRMOD)
        //      OXTABS  = expand tabs to spaces
        //      ONOEOT  = discard EOT's (^D) on output
        //      OCRNL   = map CR to NL
        //      OLCUC   = translate lower case to upper case
        //      ONOCR   = no CR output at column 0
        //      ONLRET  = NL performs CR function
        tios.c_oflag = 0; // disable all the output processing

        // CONTROL MODES:
        //  c_cflag
        //      CSIZE   = character size mask
        //      CS8
        //      CSTOPB  = send 2 stop bits
        //      CREAD   = enable receiver
        //      PARENB  = parity enable
        //      PARODD  = odd parity, else even
        //      HUPCL   = hang up on last close
        //      CLOCAL  = ignore modem status lines
        //      CCTS_OFLOW  = CTS flow control of output
        //      CRTSCTS = same as CCTS_OFLOW
        //      CRTS_IFLOW  = RTS flow control of input
        //      MDMBUF  = flow control output via character
        tios.c_cflag &= !(libc::CSIZE); // 8
        tios.c_cflag |= libc::CS8;
        tios.c_cflag &= !(libc::PARENB); // N
        tios.c_cflag &= !(libc::CSTOPB); // 1
                                         // disable hardware flow control
        tios.c_cflag &= !(libc::CRTSCTS);
        // enable receiver & ignore modem control lines
        tios.c_cflag |= libc::CREAD | libc::CLOCAL;

        // LOCAL MODES:
        //  c_lflag
        //      ECHOKE  = visual erase for line kill
        //      ECHOE   = visually erase chars
        //      ECHO    = enable echoing
        //      ECHNOL  = echo NL even if ECHO is off
        //      ECHOPRT = visual eerase mode for hardcopy
        //      ECHOCTL = echo control chars as ^Char
        //      ISIG    = enable signals INTR QUIT [D]SUSP
        //      ICANON  = canonicalize input lines
        //      ALTWERASE   = use alternate WERASE algorithm
        //      IEXTEN  = enable DISCARD and LNEXT
        //      EXTPROC = external processing
        //      TOSTOP  = stop background jobs from output
        //      FLUSHO  = output being flushed (state)
        //      NOKERNINFO  = no kernel output from VSTATUS
        //      PENDIN  = XXX retype pending input (state)
        //      NOFLSH  = don't flush after interrupt
        tios.c_lflag = 0; // disable all local modes

        // mode: MIN=0 TIME=0:
        //  minimum of either the number of bytes requested or the number of bytes currently
        //  available is returned without waiting for more bytes to be input. If no characters are
        //  available, read returns a value of zero, having read no data.
        tios.c_cc[libc::VMIN] = 0;
        tios.c_cc[libc::VTIME] = 0;

        // FIX: for higher speeds, tcsetattr will EINVAL if it gets ispeed/ospeed out of the
        // permitted range; these values are arbitrary, we will rewrite them later
        tios.c_ospeed = libc::B115200;
        tios.c_ispeed = libc::B115200;

        let tcsa_mode = if drain {
            libc::TCSADRAIN
        } else {
            libc::TCSANOW
        };
        // SAFETY:
        let r = unsafe { libc::tcsetattr(self.fd, tcsa_mode, &raw const tios) };
        if r != 0 {
            eyre::bail!("failed to tcsetattr: error={}", nix::errno::Errno::last());
        }

        let speed = baud as libc::speed_t;

        // FIX: we end up needing to ignore errors from iossiospeed
        // XXX(mc): seems like we aren't actually ignoring them?
        // XXX(mc): nvm, iossiospeed fails if it's a PTY, which is the case when using socat for
        //          debugging
        unsafe {
            let _ = iossiospeed(self.fd, &raw const speed);
            // .map_err(|c_err| eyre::eyre!("failed to set speed (iossiospeed): {}", c_err))?
        };
        Ok(())
    }
}

fn pipe_error(e: &io::Error) {
    if e.kind() == ErrorKind::BrokenPipe {
        tracing::error!("device disconnected. aborting.");
        exit(1);
    }
}
impl Tty {
    pub fn write32_le(&mut self, w: u32) -> io::Result<()> {
        self.write_all(&u32::to_le_bytes(w))
    }

    pub fn read32_le(&mut self) -> io::Result<u32> {
        let mut buf = [0; 4];
        self.read_exact(&mut buf).inspect_err(pipe_error)?;
        Ok(u32::from_le_bytes(buf))
    }

    pub fn read8(&mut self) -> io::Result<u8> {
        let mut buf: [u8; 1] = [0];
        self.read_exact(&mut buf).inspect_err(pipe_error)?;
        Ok(buf[0])
    }
}
