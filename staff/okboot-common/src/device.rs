use crate::{EncodeMessageType, MessageType};
use serde::{Deserialize, Serialize};
// /// Send a string to the host to be printed out. Messages will be line-buffered in a timeout-limited
// /// manner.
// #[derive(Debug, Serialize, Deserialize)]
// #[repr(C)]
// pub struct PrintString<'a> {
//     pub string: &'a str,
// }

/// Indicates the protocol versions that the device can speak.
// This requires a bit of song-and-dance because of a limitation of musli.
// Specifically, we can't [`Deserialize`] &[u32].
// As far as I can tell, this would require some alignment guarantees that it doesn't want to make.
// We *could* try to make those guarantees externally, and then use bytemuck + byteorder internally
// But I think this is more robust.
// `versions` only needs to be read on the host-side, and there, we only need to operate on the
// iterator from this.
#[derive(Debug, Serialize, Deserialize)]
#[repr(C)]
pub struct AllowedVersions<'a> {
    versions: &'a [u8],
}
pub struct AllowedVersionsIter<'a, 'b> {
    offset: usize,
    parent: &'a AllowedVersions<'b>,
}
impl<'a, 'b> Iterator for AllowedVersionsIter<'a, 'b> {
    type Item = u32;
    fn next(&mut self) -> Option<Self::Item> {
        if self.offset >= self.parent.versions.len() {
            None
        } else if self.offset + 4 > self.parent.versions.len() {
            // technically, unreachable, but whatever
            // this gives us better codegen
            None
        } else {
            let four_bytes: [u8; 4] = self.parent.versions[self.offset..self.offset + 4]
                .try_into()
                .expect("impossible");
            self.offset += 4;
            Some(u32::from_le_bytes(four_bytes))
        }
    }
}
impl<'a> AllowedVersions<'a> {
    pub fn new(versions: &'a [u32]) -> Self {
        Self {
            versions: bytemuck::must_cast_slice(versions),
        }
    }
    pub fn iter<'s>(&'s self) -> AllowedVersionsIter<'s, 'a> {
        AllowedVersionsIter {
            offset: 0,
            parent: self,
        }
    }
}
impl EncodeMessageType for AllowedVersions<'_> {
    const TYPE: MessageType = MessageType::AllowedVersions;
}

/// Signals that the host should send program [`Metadata`](crate::host::Metadata).
#[derive(Debug, Serialize, Deserialize)]
#[repr(C)]
pub struct MetadataReq {}
impl EncodeMessageType for MetadataReq {
    const TYPE: MessageType = MessageType::MetadataReq;
}

/// Retransmission (for confirmation) of metadata information, and the chunk size that the host
/// should use for transmission.
#[derive(Debug, Serialize, Deserialize)]
#[repr(C)]
pub struct MetadataAck {
    pub chunk_size: u32,
    pub metadata: crate::host::Metadata,
}
impl EncodeMessageType for MetadataAck {
    const TYPE: MessageType = MessageType::MetadataAck;
}

/// Request a specific chunk from the host.
#[derive(Debug, Serialize, Deserialize)]
#[repr(C)]
pub struct ChunkReq {
    pub which: u32,
}
impl EncodeMessageType for ChunkReq {
    const TYPE: MessageType = MessageType::ChunkReq;
}

/// Indicate that the device has finished downloading.
#[derive(Debug, Serialize, Deserialize)]
#[repr(C)]
pub struct Booting {}
impl EncodeMessageType for Booting {
    const TYPE: MessageType = MessageType::Booting;
}
