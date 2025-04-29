//! ELF to UF2 converter for RPI PICO 2 / RISCV.

use clap::Parser;
use elf::ElfBytes;
use elf::abi::SHT_NOBITS;
use eyre::Result;
use std::path::Path;

fn main() {
    let args = Args::parse();
    let in_path = Path::new(&args.input);
    if !in_path.is_file() {
        panic!("Input file does not exist.");
    }
    let out_path = Path::new(&args.output);
    let input = std::fs::read(in_path).expect("Failed to read input file.");
    let output = encode_uf2(&input).expect("Failed to generate UF2.");
    std::fs::write(out_path, &output).expect("Failed to write output file.");
}

/// Command-line arguments.
#[derive(Parser)]
struct Args {
    /// Path where resulting UF2 will be written to.
    #[arg(short = 'o', required = true)]
    output: String,
    /// Path where ELF will be read from.
    input: String,
}

// -- SECTION: UF2 Binary Format

proc_bitfield::bitfield! {
    /// Flag structure used by UF2 blocks.
    #[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
    pub struct Uf2Flags(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        extension_tags_present: bool @ 15,
        md5_present: bool @ 14,
        family_id_present: bool @ 13,
        file_container: bool @ 12,
        not_main_flash: bool @ 0,
    }
}
impl Uf2Flags {
    pub const EMPTY: Self = Self(0);
}

/// End of XIP flash memory on RP2350
const RP2350_END_OF_FLASH: u32 = 0x1100_0000;

const RP2350_UF2_PAYLOAD_SIZE: usize = 256;

/// UF2 family IDs used by RP2350 Bootrom. There are others, but these are the two we need.
///
/// See Reference Manual §5.5.3 on pg. 400, "UF2 Targeting Rules".
#[derive(Debug, Copy, Clone)]
#[repr(u32)]
pub enum FamilyId {
    Absolute = 0xe48bff57,
    Rp2350Riscv = 0xe48bff5a,
}

/// Maximum amount of data that can be contained in a UF2 block.
const UF2_BLOCK_DATA_SIZE: usize = 476;

/// UF2 block. Layout in memory is correct to the specification, and so a `&Uf2Block` can be
/// directly cast to a byte slice and written out (using e.g. [`bytemuck::bytes_of`]).
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
struct Uf2Block {
    magic0: u32,
    magic1: u32,

    flags: Uf2Flags,
    target_address: u32,
    payload_size: u32,
    block_no: u32,
    block_count: u32,
    file_size_family_id: u32,
    data: [u8; UF2_BLOCK_DATA_SIZE],

    magic2: u32,
}
impl Uf2Block {
    // Magic constants used by UF2 format
    const MAGIC0: u32 = 0x0a324655;
    const MAGIC1: u32 = 0x9e5d5157;
    const MAGIC2: u32 = 0x0ab16f30;

    pub fn new(
        flags: Uf2Flags,
        target_address: u32,
        block_no: u32,
        block_count: u32,
        family_id: FamilyId,
        extensions: Option<&[u8]>,
        data: &[u8],
    ) -> Self {
        let extensions = extensions.unwrap_or(&[]);
        assert!(
            data.len() + extensions.len() < UF2_BLOCK_DATA_SIZE,
            "Data and extensions too long for UF2 block"
        );
        let mut this = Self {
            magic0: Self::MAGIC0,
            magic1: Self::MAGIC1,
            flags: flags
                .with_family_id_present(true)
                .with_extension_tags_present(!extensions.is_empty()),
            target_address,
            payload_size: data.len().try_into().unwrap(),
            block_no,
            block_count,
            file_size_family_id: family_id as u32,
            data: [0; UF2_BLOCK_DATA_SIZE],
            magic2: Self::MAGIC2,
        };
        this.data[0..data.len()].copy_from_slice(data);
        this.data[data.len()..data.len() + extensions.len()].copy_from_slice(extensions);
        this
    }
}

// -- SECTION: ELF to UF2 Conversion

/// Intermediate representation of part of an ELF segment.
struct IBlock {
    flags: Uf2Flags,
    target_address: u32,
    family_id: FamilyId,
    data: Vec<u8>,
}

/// Convert an ELF file to a UF2 file targeted at rp2350-riscv.
pub fn encode_uf2(file: &[u8]) -> Result<Vec<u8>> {
    // Parse ELF
    let elf: ElfBytes<elf::endian::LittleEndian> =
        ElfBytes::minimal_parse(file).expect("Failed to parse ELF file (expected: elf32lriscv).");
    let Some(segment_table) = elf.segments() else {
        eyre::bail!("no ELF segment table");
    };

    // This will hold the generated UF2
    let mut out = vec![];

    // Errata: RP2350-E10
    out.extend_from_slice(bytemuck::bytes_of(&Uf2Block::new(
        Uf2Flags::EMPTY,
        RP2350_END_OF_FLASH - RP2350_UF2_PAYLOAD_SIZE as u32,
        0,
        2,
        FamilyId::Absolute,
        Some(&[0x04, 0xe3, 0x57, 0x99]),
        &[0xe5; RP2350_UF2_PAYLOAD_SIZE],
    )));

    // (normal) block generation is more or less a two-step process, since we need to know, before
    // writing out the first block, the total number of blocks in the file we're writing out, so we
    // first parse the ELF segment table to find all loadable segments, turn them into simplified
    // `IBlock`s, count them, and then write them out.

    let mut iblocks = vec![];
    for segment in segment_table.iter() {
        match segment.p_type {
            elf::abi::PT_LOAD => {
                // Don't waste valuable flash space on BSS
                if (segment.p_flags & SHT_NOBITS) == SHT_NOBITS {
                    continue;
                }
                let file_bytes = &file
                    [segment.p_offset as usize..(segment.p_offset + segment.p_filesz) as usize];
                // use VA for target address; we typically want to ignore PA
                let target_address = u32::try_from(segment.p_vaddr)?;
                for (j, chunk) in file_bytes.chunks(RP2350_UF2_PAYLOAD_SIZE).enumerate() {
                    let mut chunk = chunk.to_vec();
                    // if this is the last chunk, and it's not 256 bytes, pad it out
                    chunk.extend(std::iter::repeat_n(
                        0u8,
                        RP2350_UF2_PAYLOAD_SIZE - chunk.len(),
                    ));
                    iblocks.push(IBlock {
                        flags: Uf2Flags::EMPTY,
                        target_address: target_address + (j * RP2350_UF2_PAYLOAD_SIZE) as u32,
                        family_id: FamilyId::Rp2350Riscv,
                        data: chunk.to_vec(),
                    });
                }
            }
            elf::abi::PT_RISCV_ATTRIBUTES => {
                // ignore
            }
            p_type => {
                eyre::bail!("unsupported PT_TYPE {:x}", p_type);
            }
        }
    }

    // Count blocks
    let total_block_count: u32 = iblocks
        .len()
        .try_into()
        .expect("Too many blocks: overflowed u32");

    // Write blocks out
    for (i, iblock) in iblocks.into_iter().enumerate() {
        out.extend_from_slice(bytemuck::bytes_of(&Uf2Block::new(
            iblock.flags,
            iblock.target_address,
            i.try_into()?,
            total_block_count,
            iblock.family_id,
            None,
            &iblock.data,
        )));
    }

    // Return generated UF2
    Ok(out)
}
