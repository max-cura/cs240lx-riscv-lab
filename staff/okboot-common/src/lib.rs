#![allow(internal_features)]
#![feature(core_intrinsics)]
#![cfg_attr(not(feature = "std"), no_std)]
//! Common structures shared by the `okboot` and `okdude` crates.
#[cfg(test)]
extern crate std;

use serde::{Deserialize, Serialize};

/// The baud rate at which the protocol is run.
pub const BAUD_RATE: usize = 1_500_000;

/// The COBS xor factor.
pub const COBS_XOR: u8 = 0x55;

/// Message preamble, shortened from Ethernet.
pub const PREAMBLE_BYTES: [u8; 4] = [0x55, 0x55, 0x55, 0x5e];

/// Message structures sent from the device.
pub mod device;
/// Frame encoding and decoding, for both sides.
pub mod frame;
/// Message structure sent from the host.
pub mod host;

pub trait EncodeMessageType {
    const TYPE: MessageType;
}

/// Message type enumeration.
#[derive(Debug, Serialize, Deserialize, Clone, Copy, Eq, PartialEq)]
#[repr(u32)]
pub enum MessageType {
    /// Corresponds to an unserialized UTF-8 string type.
    PrintString = 101,
    /// Corresponds to [`Probe`](host::Probe)
    Probe = 201,
    /// Corresponds to [`AllowedVersions`](device::AllowedVersions).
    AllowedVersions = 202,
    /// Corresponds to [`UseVersion`](host::UseVersion)
    UseVersion = 203,
    /// Corresponds to [`MetadataReq`](device::MetadataReq)
    MetadataReq = 301,
    /// Corresponds to [`Metadata`](host::Metadata)
    Metadata = 302,
    /// Corresponds to [`MetadataAck`](device::MetadataAck)
    MetadataAck = 303,
    /// Corresponds to [`MetadataAckAck`](host::MetadataAckAck)
    MetadataAckAck = 304,
    /// Corresponds to [`ChunkReq`](device::ChunkReq`)
    ChunkReq = 401,
    /// Corresponds to [`Chunk`](host::Chunk)
    Chunk = 402,
    /// Corresponds to [`Booting`](device::Booting)
    Booting = 501,
    /// Corresponds to [`BootingAck`](host::BootingAck)
    BootingAck = 502,
}
impl From<MessageType> for u32 {
    fn from(val: MessageType) -> u32 {
        val as u32
    }
}
impl TryFrom<u32> for MessageType {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(match value {
            101 => Self::PrintString,
            201 => Self::Probe,
            202 => Self::AllowedVersions,
            203 => Self::UseVersion,
            301 => Self::MetadataReq,
            302 => Self::Metadata,
            303 => Self::MetadataAck,
            304 => Self::MetadataAckAck,
            401 => Self::ChunkReq,
            402 => Self::Chunk,
            501 => Self::Booting,
            502 => Self::BootingAck,
            _ => return Err(()),
        })
    }
}

pub const INITIAL_BAUD_RATE: u32 = 115200;

pub mod su_boot {

    #[derive(Debug, Copy, Clone, Eq, PartialEq)]
    #[repr(u32)]
    pub enum Command {
        BootStart = 0xFFFF0000,

        GetProgInfo = 0x11112222, // pi sends
        PutProgInfo = 0x33334444, // unix sends

        GetCode = 0x55556666, // pi sends
        PutCode = 0x77778888, // unix sends

        BootSuccess = 0x9999AAAA, // pi sends on success
        BootError = 0xBBBBCCCC,   // pi sends on failure.

        PrintString = 0xDDDDEEEE, // pi sends to print a string.
    }
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum SupportedProtocol {
    V2 = 2,
}
impl TryFrom<u32> for SupportedProtocol {
    type Error = u32;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            2 => Ok(Self::V2),
            _ => Err(value),
        }
    }
}
impl SupportedProtocol {
    pub fn baud_rate(self) -> u32 {
        match self {
            SupportedProtocol::V2 => 115200,
        }
    }
}
