//! Support infrastructure to allow booting this crate from a UF2 file stored to the PICO's
//! USB Mass Media drive.

#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
#[allow(dead_code)]
struct ImageDef {
    typ: u8,
    size: u8,
    flags: u16,
}
#[derive(Debug, Copy, Clone)]
#[repr(C, packed)]
#[allow(dead_code)]
struct LastItem {
    typ: u8,
    size: u16,
    pad: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
union BlockItem {
    image_def: ImageDef,
    last_item: LastItem,
}
const _: () = const { assert!(size_of::<BlockItem>() == 4) };
#[derive(Copy, Clone)]
#[allow(dead_code)]
#[repr(C, align(4))]
struct Block<const N: usize> {
    header: u32,
    items: [BlockItem; N],
    next: u32,
    footer: u32,
}
#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".rp2350.boot_header")]
static BOOTROM_BLOCK_LOOP: Block<2> = Block {
    header: 0xffff_ded3,
    items: [
        BlockItem {
            image_def: ImageDef {
                typ: 0x42,
                size: 1,
                flags: 0x1101,
            },
        },
        BlockItem {
            last_item: LastItem {
                typ: 0xff,
                size: 0x0001,
                pad: 0x00,
            },
        },
    ],
    next: 0,
    footer: 0xab12_3579,
};
