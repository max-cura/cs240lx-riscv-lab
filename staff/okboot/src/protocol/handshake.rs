use crate::buf::FrameSink;
use crate::legacy_print_string;
use crate::protocol::{Protocol, ProtocolEnum, ProtocolStatus, Timeouts};
// use bcm2835_lpa::Peripherals;
use okboot_common::device::AllowedVersions;
use okboot_common::frame::FrameHeader;
use okboot_common::host::UseVersion;
use okboot_common::{MessageType, SupportedProtocol};
use quartz::device::timing::{FloatingTimer, delay_millis};
use quartz::device::uart::Uart;
// use quartz::device::bcm2835::mini_uart::baud_to_clock_divider;

const SUPPORTED_PROTOCOL_VERSIONS: &[u32] = &[okboot_common::SupportedProtocol::V2 as u32];

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum Expecting {
    Probe,
    Version,
}

#[derive(Debug)]
pub struct Handshake {
    expecting: Expecting,
}
impl Default for Handshake {
    fn default() -> Self {
        Self {
            expecting: Expecting::Probe,
        }
    }
}
impl Protocol for Handshake {
    fn handle_packet<U: Uart, FT: FloatingTimer>(
        &mut self,
        frame_header: FrameHeader,
        payload: &[u8],
        frame_sink: &mut FrameSink,
        _timeouts: &mut Timeouts,
        uart: &U,
        ft: &FT,
        // _inflate_buffer: &mut [u8],
    ) -> ProtocolStatus {
        match frame_header.message_type {
            MessageType::Probe => {
                if !matches!(self.expecting, Expecting::Probe) {
                    legacy_print_string!(
                        frame_sink,
                        "[device]: received Handshake/Probe, expected Handshake/UseVersion"
                    );
                    ProtocolStatus::Abcon
                } else {
                    legacy_print_string!(frame_sink, "[device]: Received Handshake/Probe");
                    self.expecting = Expecting::Version;
                    match crate::buf::send(
                        frame_sink,
                        &AllowedVersions::new(SUPPORTED_PROTOCOL_VERSIONS),
                    ) {
                        Err(e) => {
                            legacy_print_string!(
                                frame_sink,
                                "[device]: failed to send Handshake/AllowedVersions: {}",
                                e
                            );
                            ProtocolStatus::Abcon
                        }
                        Ok(()) => {
                            legacy_print_string!(
                                frame_sink,
                                "[device]: sent Handshake/AllowedVersions"
                            );
                            ProtocolStatus::Continue
                        }
                    }
                }
            }
            MessageType::UseVersion => {
                if !matches!(self.expecting, Expecting::Version) {
                    legacy_print_string!(
                        frame_sink,
                        "[device]: received Handshake/UseVersion, expected Handshake/Probe"
                    );
                    return ProtocolStatus::Abend;
                }
                legacy_print_string!(frame_sink, "[device]: received Handshake/UseVersion");
                let use_version: UseVersion = match postcard::from_bytes(payload) {
                    Ok(x) => x,
                    Err(e) => {
                        legacy_print_string!(
                            frame_sink,
                            "[device]: failed to receive Handshake/UseVersion: deserialization error: {}",
                            e
                        );
                        return ProtocolStatus::Abend;
                    }
                };
                let protocol_version = match SupportedProtocol::try_from(use_version.version) {
                    Ok(sp) => sp,
                    Err(_) => {
                        legacy_print_string!(
                            frame_sink,
                            "[device]: Handshake/UseVersion: unsupported version number: {}",
                            use_version.version
                        );
                        return ProtocolStatus::Abend;
                    }
                };

                let new_baud_rate = protocol_version.baud_rate();
                // let new_clock_divider = baud_to_clock_divider(new_baud_rate);
                legacy_print_string!(
                    frame_sink,
                    "[device]: setting baud rate to: {}Bd", //, divider={}",
                    new_baud_rate,
                    // new_clock_divider
                );

                super::flush_to_fifo(frame_sink, uart);
                uart.flush_tx();
                uart.set_baud_rate(new_baud_rate);
                // if !crate::mini_uart::mini_uart1_set_clock(&peripherals.UART1, new_clock_divider) {
                //     legacy_print_string!(
                //         frame_sink,
                //         "[device]: setting baud rate failed (divider readback failed)"
                //     );
                //     return ProtocolStatus::Abend;
                // }

                legacy_print_string!(
                    frame_sink,
                    "[device:v{}]: set baud rate to: {}Bd", //, divider={}",
                    use_version.version,
                    new_baud_rate,
                    // new_clock_divider
                );

                delay_millis(ft, 50);
                // quartz::device::bcm2835::timing::delay_millis(&peripherals.SYSTMR, 50);
                crate::rpc_println!(
                    frame_sink,
                    "[device:v{}]: transitioned baud rate!",
                    use_version.version
                );

                ProtocolStatus::Switch(ProtocolEnum::V2(super::v2::V2::new(ft, new_baud_rate)))
            }
            w => {
                legacy_print_string!(
                    frame_sink,
                    "[device]: in Handshake protocol, unexpected message type: {:?}",
                    w
                );
                ProtocolStatus::Abend
            }
        }
    }

    fn heartbeat<FT: FloatingTimer>(
        &mut self,
        _frame_sink: &mut FrameSink,
        _timeouts: &mut Timeouts,
        _ft: &FT,
    ) -> ProtocolStatus {
        ProtocolStatus::Continue
    }
}
