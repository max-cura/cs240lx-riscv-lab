unsafe extern "C" {
    static __data_start__: [u8; 0];
    static __heap_end__: [u8; 0];
}

pub unsafe fn locate_end() -> *const [u8; 0] {
    &raw const __heap_end__
}
pub unsafe fn locate_start() -> *const [u8; 0] {
    &raw const __data_start__
}

pub mod flat_binary {
    use core::arch::asm;

    use alloc::format;
    use okboot_common::INITIAL_BAUD_RATE;
    use quartz::device::uart::Uart;

    use crate::buf::FrameSink;
    // use bcm2835_lpa::Peripherals;
    // use quartz::arch::arm1176::PAGE_SIZE;
    // use quartz::arch::arm1176::mmu::__disable_mmu;

    const PAGE_SIZE: usize = 0x4000;

    #[derive(Clone, Debug)]
    pub struct Relocation {
        pub base_address_ptr: *mut u16,
        pub side_buffer_ptr: *mut u16,
        pub relocate_first_n_bytes: usize,
        // pub stub_entry: *mut [u16; 0],
        relocate: bool,
    }

    impl Relocation {
        pub fn calculate(base_address: usize, k_length: usize, self_end_addr: usize) -> Relocation {
            let k_base_address = base_address;
            let k_end_address = k_base_address + k_length;

            let needs_to_relocate = k_base_address < self_end_addr;

            let highest_used_address = self_end_addr.max(k_end_address);
            let side_buffer_begin = (highest_used_address + PAGE_SIZE - 1) & !(PAGE_SIZE - 1);

            if needs_to_relocate {
                let relocation_length = k_end_address.min(self_end_addr) - k_base_address;
                // need this to be 4-byte aligned if we want to jump to it
                // let stub_location = (side_buffer_begin + relocation_length + 3) & !3;
                Relocation {
                    base_address_ptr: k_base_address as *mut u16,
                    side_buffer_ptr: side_buffer_begin as *mut u16,
                    relocate_first_n_bytes: relocation_length,
                    // stub_entry: stub_location as *mut [u16; 0],
                    relocate: true,
                }
            } else {
                Relocation {
                    base_address_ptr: core::ptr::null_mut(),
                    side_buffer_ptr: core::ptr::null_mut(),
                    relocate_first_n_bytes: 0,
                    // stub_entry: highest_used_address as *mut [u16; 0],
                    relocate: false,
                }
            }
        }

        pub unsafe fn write_bytes(&self, address: *mut u8, bytes: &[u8]) {
            let (ptr, len) = (bytes.as_ptr(), bytes.len());
            let write_ptr = if self.relocate
                && address >= self.base_address_ptr.cast::<u8>()
                && address
                    < unsafe {
                        self.base_address_ptr
                            .cast::<u8>()
                            .byte_offset(self.relocate_first_n_bytes as isize)
                    } {
                unsafe {
                    self.side_buffer_ptr
                        .cast::<u8>()
                        .byte_offset(address.byte_offset_from(self.base_address_ptr))
                }
            } else {
                address
            };
            unsafe { core::ptr::copy(ptr, write_ptr, len) };
        }

        pub unsafe fn verify_integrity(&self, expected_crc: u32, len: usize) -> Integrity {
            // let peri = unsafe { Peripherals::steal() };
            let mut hasher = crc32fast::Hasher::new();
            // crate::print_rpc!(fs, "[device:v1]: verifying integrity (1)");
            // fs._flush_to_fifo(&rz.peri.UART1);
            if self.relocate {
                // crate::print_rpc!(fs, "[device:v1]: verifying integrity (2)");
                // fs._flush_to_fifo(&rz.peri.UART1);
                let side_buf = unsafe {
                    core::slice::from_raw_parts(
                        self.side_buffer_ptr.cast::<u8>(),
                        self.relocate_first_n_bytes,
                    )
                };
                // legacy_print_string_blocking!(&peri.UART1, "[v2/rel] crc buf {:02x?}", side_buf,);
                hasher.update(side_buf);
            }
            // let a = self.base_address_ptr.byte_offset(self.relocate_first_n_bytes as isize);
            // let b = len - self.relocate_first_n_bytes;
            // crate::print_rpc!(fs, "[device:v1]: verifying integrity (3) / {len}:{} / {a:#?}:{b}", self.relocate_first_n_bytes);
            // fs._flush_to_fifo(&rz.peri.UART1);
            if len > self.relocate_first_n_bytes {
                let inplace_buf = unsafe {
                    core::slice::from_raw_parts(
                        self.base_address_ptr
                            .cast::<u8>()
                            .byte_offset(self.relocate_first_n_bytes as isize),
                        len - self.relocate_first_n_bytes,
                    )
                };
                // legacy_print_string_blocking!(&peri.UART1, "[v2/rel] crc buf {:02x?}", inplace_buf,);
                hasher.update(inplace_buf);
            }
            // crate::print_rpc!(fs, "[device:v1]: verifying integrity (4)");
            // fs._flush_to_fifo(&rz.peri.UART1);

            let final_crc = hasher.finalize();

            if expected_crc == final_crc {
                Integrity::Ok
            } else {
                Integrity::CrcMismatch {
                    expected: expected_crc,
                    calculated: final_crc,
                }
            }
        }
    }

    pub enum Integrity {
        Ok,
        CrcMismatch { expected: u32, calculated: u32 },
    }

    pub unsafe fn final_relocation<U: Uart>(
        fs: &mut FrameSink,
        relocation: Relocation,
        uart: &U,
    ) -> ! {
        // let peripherals = unsafe { rp235x_pac::Peripherals::steal() };
        // let uart = quartz::device::rp2350::PL011Uart::init(
        //     115200,
        //     150_000_000,
        //     &peripherals.UART0,
        //     &peripherals.RESETS,
        // );
        crate::protocol::flush_to_fifo(fs, uart);
        uart.flush_tx();
        uart.set_baud_rate(INITIAL_BAUD_RATE);
        // let ft = quartz::device::rp2350::RiscvTimer::new();
        // delay_millis(&ft, 1000);
        for _ in 0..1_000_000 {
            unsafe { asm!("nop") }
        }

        macro_rules! print {
            ($out:expr, $($t:tt)*) => {
                crate::legacy::uart1::uart1_write_bytes(
                    $out,
                    format!($($t)*).as_bytes(),
                );
                crate::legacy::uart1::uart1_write_bytes($out, b"\r\n");
            }
        }
        // let stub_dst = relocation.stub_entry;
        let kernel_dst = relocation.base_address_ptr;
        let kernel_src = relocation.side_buffer_ptr;
        let kernel_copy_len = relocation.relocate_first_n_bytes;
        let kernel_entry = relocation.base_address_ptr.cast();

        // let stub_begin = &raw const __symbol_relocation_stub;
        // let stub_end = &raw const __symbol_relocation_stub_end;

        // let stub_len = unsafe { stub_end.byte_offset_from(stub_begin) as usize };

        print!(uart, "[device:v1]: relocation_stub parameters:");
        // crate::legacy_print_string_blocking!(uart, "\tstub destination={stub_dst:#?}");
        // crate::legacy_print_string_blocking!(uart, "\tstub code={stub_begin:#?}");
        // crate::legacy_print_string_blocking!(uart, "\tstub length={stub_len:#?}");
        print!(uart, "\tcopy to={kernel_dst:#?}");
        print!(uart, "\tcopy from={kernel_src:#?}");
        print!(uart, "\tcopy bytes={kernel_copy_len}");
        print!(uart, "\tentry={kernel_entry:#?}");

        // unsafe {
        // core::ptr::copy(stub_begin as *const u8, stub_dst, stub_len);
        // }

        print!(uart, "[device:v1]: Loaded relocation-stub, jumping");

        // crate::protocol::flush_to_fifo(fs, uart);
        uart.flush_tx();
        // crate::mini_uart::mini_uart1_flush_tx(&peripherals.UART1);

        // led_set(true);

        // unsafe { __disable_mmu() };

        // unsafe {
        //     core::arch::asm!(
        //     "jr {t0}",
        //     in("a0") kernel_dst,
        //     in("a1") kernel_src,
        //     in("a2") kernel_copy_len,
        //     in("a3") kernel_entry,
        //     t0 = in(reg) stub_dst,
        //     options(noreturn),
        //     )
        // }
        // loop {}
        unsafe { perform_final_relocation(kernel_dst, kernel_src, kernel_copy_len, kernel_entry) }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn perform_final_relocation(
        dst: *mut u16,
        src: *const u16,
        len: usize,
        entry: *mut [u16; 0],
    ) -> ! {
        unsafe {
            asm!(
                r#"
                    // li s2, 0xd0000000
                    // li s3, 1 << 25
                    // sw s3, 0x030(s2)

                    andi t0, a2, -0x4
                    beqz t0, 3f
                2:
                    lw t1, 0(a1)
                    sw t1, 0(a0)
                    addi a1, a1, 4
                    addi a0, a0, 4
                    addi t0, t0, -4
                    bnez t0, 2b
                3:
                    andi t0, a2, 0x3
                    beqz t0, 5f
                4:
                    lb t1, 0(a1)
                    sb t1, 0(a0)
                    addi a1, a1, 1
                    addi a0, a0, 1
                    addi t0, t0, -1
                    bnez t0, 4b
                5:
                    fence.i
                    jr a3
                "#,
                in("a0") dst,
                in("a1") src,
                in("a2") len,
                in("a3") entry,
                options(noreturn)
            )
        }
    }
}

// unsafe extern "C" {
//     pub(crate) static __symbol_relocation_elf: [u8; 0];
//     pub(crate) static __symbol_relocation_elf_end: [u8; 0];
// }

pub mod elf {
    use crate::buf::FrameSink;
    use crate::legacy_print_string_blocking;
    use crate::stub::{locate_end, locate_start};
    use alloc::vec;
    use alloc::vec::Vec;
    use quartz::device::timing::delay_millis;
    use quartz::device::uart::Uart;
    // use bcm2835_lpa::Peripherals;
    use core::range::Range;
    use core::slice::GetDisjointMutIndex;
    use elf::segment::Elf32_Phdr;
    // use quartz::arch::arm1176::dsb;
    // use quartz::arch::arm1176::mmu::__disable_mmu;
    // use quartz::device::bcm2835::mini_uart::{baud_to_clock_divider, muart1_init};
    // use quartz::device::bcm2835::timing::delay_millis;

    pub unsafe fn final_relocation(
        // peripherals: &Peripherals,
        fs: &mut FrameSink,
        pheaders: Vec<Elf32_Phdr>,
        elf: &[u8],
        entry: usize,
    ) -> ! {
        // muart1_init(
        //     &peripherals.GPIO,
        //     &peripherals.AUX,
        //     &peripherals.UART1,
        //     270, // baud_to_clock_divider(okboot_common::BAUD_RATE as u32),
        // );
        let peripherals = unsafe { rp235x_pac::Peripherals::steal() };
        let uart = quartz::device::rp2350::PL011Uart::init(
            115200,
            150_000_000,
            &peripherals.UART0,
            &peripherals.RESETS,
        );
        let ft = quartz::device::rp2350::RiscvTimer::new();

        delay_millis(&ft, 1000);
        // delay_millis(&peripherals.SYSTMR, 1000);
        // let stub_begin = &raw const __symbol_relocation_elf;
        // let stub_end = &raw const __symbol_relocation_elf_end;

        // dsb();
        // peripherals
        //     .GPIO
        //     .gpfsel2()
        //     .modify(|_, w| w.fsel27().output());
        // // peripherals
        // //     .GPIO
        // //     .gpset0()
        // //     .write_with_zero(|w| w.set27().set_bit());
        // dsb();

        // let stub_len = unsafe { stub_end.byte_offset_from(stub_begin) as usize };
        // legacy_print_string_blocking!(&uart, "\t[elf] stub code={stub_begin:#?}\n");
        // legacy_print_string_blocking!(&uart, "\t[elf] stub length={stub_len:#?}\n");

        let ram_start = unsafe { locate_start().addr() };
        let ram_end = unsafe { locate_end().addr() };

        let mut gaps: Vec<Range<usize>> = vec![(ram_start..ram_end).into()];
        for pheader in &pheaders {
            let start = pheader.p_vaddr as usize;
            let end = start + pheader.p_memsz as usize;
            let hdr = (start..end).into();
            let mut remove = Vec::new();
            let mut add: Vec<Range<usize>> = Vec::new();
            for (i, gap) in gaps.iter().enumerate() {
                if gap.is_overlapping(&hdr) {
                    remove.push(i);
                    if gap.start < hdr.start {
                        add.push((gap.start..hdr.start).into());
                    }
                    if hdr.end < gap.end {
                        add.push((hdr.end..gap.end).into());
                    }
                }
            }
            remove.sort();
            remove.reverse();
            for i in remove.iter() {
                gaps.swap_remove(*i);
            }
            gaps.extend(add);
        }
        let Some(gap) = gaps
            .into_iter()
            .find(|gap| (gap.end - gap.start) >= core::mem::size_of_val(&pheaders))
        else {
            legacy_print_string_blocking!(
                &uart,
                "-- ERROR: no gap sufficient to fit in ELF program header chain for final relocation"
            );
            panic!("ERROR: failedd to relocate program header chain");
        };
        let pheader_target = gap.start;

        let (phdr_ptr, phdr_len, _phdr_cap) = pheaders.into_raw_parts();
        let elf_base = elf.as_ptr();

        legacy_print_string_blocking!(&uart, "\t[elf] ELF base={elf_base:#?}\n");
        legacy_print_string_blocking!(&uart, "\t[elf] ELF entry={entry:#?}\n");
        legacy_print_string_blocking!(&uart, "\t[elf] program headers={phdr_ptr:#?}\n");
        legacy_print_string_blocking!(&uart, "\t[elf] program header count={phdr_len:#?}\n");
        legacy_print_string_blocking!(
            &uart,
            "\t[elf] program header destination={pheader_target:08x}\n"
        );

        // let stub_layout = Layout::from_size_align(stub_len, 0x20).unwrap();
        // let stub_dst = unsafe { alloc::alloc::alloc(stub_layout) };
        // legacy_print_string_blocking!(&uart, "\t[elf] stub dst={stub_dst:#?}\n");

        // unsafe { core::ptr::copy(stub_begin.cast(), stub_dst, stub_len) };

        crate::protocol::flush_to_fifo(fs, &uart);
        uart.flush_tx();
        // crate::mini_uart::mini_uart1_flush_tx(&peripherals.UART1);

        // unsafe { __disable_mmu() };

        // unsafe {
        //     asm!(
        //     "jr {t0}",
        //     in("a0") phdr_ptr,
        //     in("a1") phdr_len,
        //     in("a2") elf_base,
        //     in("a3") entry,
        //     t0 = in(reg) stub_dst,
        //     options(noreturn),
        //     )
        // }
        panic!("not implemented");

        // #[unsafe(naked)]
        // pub unsafe extern "C" fn perform_final_relocation(
        //     phdr_ptr: *mut Elf32_Phdr,
        //     phdr_len: usize,
        //     elf_base: *mut [u16; 0],
        //     entry: *mut [u16; 0],
        // ) -> ! {
        //     unsafe {
        //         naked_asm!(
        //             // inputs:
        //             //   a0 = phdr_ptr
        //             //   a1 = phdr_len
        //             //   a2 = elf_base
        //             //   a3 = entry
        //             r#"

        //         _elfreloc_memcpy:
        //             andi t0, a2, -0xf
        //             beqz t0, _elfreloc_memcpy_fine
        //         _elfreloc_memcpy_coarse_loop:
        //             lw t1, 0(a1)
        //             sw t1, 0(a0)
        //             addi a1, a1, 4
        //             addi a0, a0, 4
        //             addi t0, t0, -4
        //             bnez t0, _elfreloc_memcpy_coarse_loop
        //         _elfreloc_memcpy_fine:
        //             andi t0, a2, 0xf
        //         _elfreloc_memcpy_fine_loop:
        //             lb t1, 0(a1)
        //             sb t1, 0(a0)
        //             addi a1, a1, 1
        //             addi a0, a0, 1
        //             addi t0, t0, -1
        //             bnez t0, _elfreloc_memcpy_fine_loop
        //             ret
        //         _elfreloc_postreloc:
        //             fence.i
        //             jr a3
        //         "#,
        //         )
        //     }
        // }
    }
}
