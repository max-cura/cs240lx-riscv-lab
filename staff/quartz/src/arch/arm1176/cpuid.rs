use crate::define_coprocessor_registers;
use core::fmt::{Display, Formatter};

define_coprocessor_registers! {
    /* Primary identification */
    [safe read] main_id : MainId => p15 0 c0 c0 0;
    [safe read] cache_type : CacheType => p15 0 c0 c0 1;
    [safe read] tcm_status : TcmStatus => p15 0 c0 c0 2;
    [safe read] tlb_type : TlbType => p15 0 c0 c0 3;
    /* Processor features */
    [safe read] processor_feature_0 : ProcessorFeature0 => p15 0 c0 c1 0;
    [safe read] processor_feature_1 : ProcessorFeature1 => p15 0 c0 c1 1;
    /* Misc */
    [safe read] debug_feature_0 => p15 0 c0 c1 2;
    [safe read] auxiliary_feature_0 => p15 0 c0 c1 3;
    /* Memory model */
    [safe read] memory_model_feature_0 : MemoryModelFeature0 => p15 0 c0 c1 4;
    [safe read] memory_model_feature_1 : MemoryModelFeature1 => p15 0 c0 c1 5;
    [safe read] memory_model_feature_2 : MemoryModelFeature2 => p15 0 c0 c1 6;
    [safe read] memory_model_feature_3 : MemoryModelFeature3 => p15 0 c0 c1 7;
    /* Instructions */
    [safe read] isa_feature_0 : IsaFeature0 => p15 0 c0 c2 0;
    [safe read] isa_feature_1 : IsaFeature1 => p15 0 c0 c1 1;
    [safe read] isa_feature_2 : IsaFeature1 => p15 0 c0 c1 2;
    [safe read] isa_feature_3 : IsaFeature3 => p15 0 c0 c1 3;
    [safe read] isa_feature_4 : IsaFeature4 => p15 0 c0 c1 4;
    [safe read] isa_feature_5 => p15 0 c0 c1 5; /* isa_feature_5 is implementation-defined */
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct MainId(pub u32): Debug, FromStorage, IntoStorage, DerefStorage {
        implementor : u8 @ 24..=31,
        variant_number: u8 @ 20..=23,
        architecture: u8 @ 16..=19,
        primary_part_number: u16 @ 4..=15,
        revision: u8 @ 0..=3,
    }
}
impl Display for MainId {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        let implementor = match self.implementor() {
            0x41 => "ARM Limited",
            _ => "unknown implementor",
        };
        let part_no_str = match self.primary_part_number() {
            0xb76 => "ARM1176JZF-S",
            _ => "unknown implementor",
        };
        write!(
            f,
            "\t{part_no_str} v{} r{}p{}, (PN {:03x}) by {implementor} ({})\n",
            self.architecture(),
            self.variant_number(),
            self.revision(),
            self.primary_part_number(),
            self.implementor()
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct CacheType(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        ctype: u8 @ 25..=28,
        separate: bool @ 24,
        dcache_page_coloring: bool @ 23,
        dcache_size: u8 @ 18..=21,
        dcache_assoc: u8 @ 15..=17,
        dcache_m: bool @ 14,
        dcache_line_len: u8 @ 12..=13,
        icache_page_coloring : bool @ 11,
        icache_size : u8 @ 6..=9,
        icache_assoc: u8 @ 3..=5,
        icache_m : bool @ 2,
        icache_line_len: u8 @ 0..=1
    }
}
impl Display for CacheType {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        if self.separate() {
            write!(
                f,
                "\tHarvard, ctype={}\n\
                 \t{}-way {}B-wide {}KB {}data cache\n\
                 \t{}-way {}B-wide {}KB {}instruction cache\n",
                match self.ctype() {
                    0b1110 => "+write back,+format C lockdown,+reg7 cache cleaning ops",
                    _ => "unknown CTYPE",
                },
                /* DATA CACHE */
                self.dcache_assoc(),
                match self.dcache_line_len() {
                    2 => "32",
                    _ => "unknown DSIZE_LEN",
                },
                match self.dcache_size() {
                    0 => "0.5",
                    1 => "1",
                    2 => "2",
                    3 => "4",
                    4 => "8",
                    5 => "16",
                    6 => "32",
                    7 => "64",
                    8 => "128",
                    _ => "unknown DSIZE_SIZE",
                },
                if self.dcache_page_coloring() {
                    "page-colored "
                } else {
                    ""
                },
                /* INSTRUCTION CACHE */
                self.icache_assoc(),
                match self.icache_line_len() {
                    2 => "32",
                    _ => "unknown ISIZE_LEN",
                },
                match self.icache_size() {
                    0 => "0.5",
                    1 => "1",
                    2 => "2",
                    3 => "4",
                    4 => "8",
                    5 => "16",
                    6 => "32",
                    7 => "64",
                    8 => "128",
                    _ => "unknown ISIZE_SIZE",
                },
                if self.icache_page_coloring() {
                    "page-colored "
                } else {
                    ""
                }
            )
        } else {
            write!(f, "single cache, raw value={:08x}", self.0)
        }
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct TcmStatus(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        dtcm: u8 @ 16..=18,
        itcm: u8 @ 0..=2,
    }
}
impl Display for TcmStatus {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tData TCM Count: {}\n \
             \tInstruction TCM Count: {}\n",
            match self.dtcm() {
                0 => "0",
                1 => "1",
                2 => "2",
                _ => "Unknown",
            },
            match self.itcm() {
                0 => "0",
                1 => "1",
                2 => "2",
                _ => "Unknown",
            }
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct TlbType(u32):  Debug, FromStorage, IntoStorage, DerefStorage {
        instruction_lockable_size: u8 @ 16..=23,
        data_lockable_size: u8 @ 8..=15,
        unified: bool @ 0,
    }
}
impl Display for TlbType {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tTLB type: {}\n",
            if !self.unified() {
                "unified"
            } else {
                "separate"
            }
        )?;
        write!(f, "\tEntries: 64x 2-way set-associative\n")?;
        if !self.unified() {
            write!(
                f,
                "\tUnified lockable TLBs: {}\n",
                self.data_lockable_size()
            )
        } else {
            write!(
                f,
                "\tInstruction lockable TLBs: {}\n\
                 \tData lockable TLBs: {}\n",
                self.instruction_lockable_size(),
                self.data_lockable_size()
            )
        }
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct ProcessorFeature0(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        state3: u8 @ 12..=15,
        state2: u8 @ 8..=11,
        state1: u8 @ 4..=7,
        state0: u8 @ 0..=3,
    }
}
impl Display for ProcessorFeature0 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\t32-bit ARM support: {}\n\tThumb support: {}\n\tJava extension interface: {}\n\tThumb-2 support: {}\n",
            match self.state0() {
                1 => "yes",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.state1() {
                1 => "thumb1 only",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.state2() {
                1 => "yes",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.state3() {
                0 => "no",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct ProcessorFeature1(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        microcontroller_programmers_model: u8 @ 8..=11,
        security_extensions_architecture_v1: u8 @ 4..=7,
        armv4_programmers_model: u8 @ 0..=3,
    }
}
impl Display for ProcessorFeature1 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tARMv4 Programmer's model: {}\n\tSecurity Extensions Architecture v1: {}\n\tARM microcontroller programmer's model: {}\n",
            match self.armv4_programmers_model() {
                1 => "supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.security_extensions_architecture_v1() {
                1 => "supported (TrustZone)",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.microcontroller_programmers_model() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct MemoryModelFeature0(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        vmsa: u8 @ 0..=3,
        pmsa: u8 @ 4..=7,
        cache_coherency_cpu_agent_shmem: u8 @ 8..=11,
        cache_coherency_dma_agent_shmem: u8 @ 12..=15,
        tcm_and_assoc_dma: u8 @ 16..=19,
        armv6_aux_ctrl_reg: u8 @ 20..=23,
        fcse: u8 @ 24..=27,
    }
}
impl Display for MemoryModelFeature0 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tVMSA: {}\n\
            \tPMSA: {}\n\
            \tCache coherency support, CPU agent, shared memory: {}\n\
            \tCache coherency support, DMA agent, shared memory: {}\n\
            \tTCM and associated DMA: {}\n\
            \tARMv6 Auxiliary Control Register: {}\n\
            \tFCSE: {}\n",
            match self.vmsa() {
                3 => "v7",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.pmsa() {
                0 => "none",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.cache_coherency_cpu_agent_shmem() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.cache_coherency_dma_agent_shmem() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.tcm_and_assoc_dma() {
                3 => "armv6",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.armv6_aux_ctrl_reg() {
                1 => "supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.fcse() {
                1 => "supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            }
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct MemoryModelFeature1(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        btb: u8 @ 28..=31,
        dcache_test_and_clean_ops: u8 @ 24..=27,
        l1_cache_all_maintenance_unified: u8 @ 20..=23,
        l1_cache_all_maintenance_harvard: u8 @ 16..=19,
        l1_cache_line_maintenance_set_way_unified: u8 @ 12..=15,
        l1_cache_line_maintenance_set_way_harvard: u8 @ 8..=11,
        l1_cache_line_maintenance_mva_unified: u8 @ 4..=7,
        l1_cache_line_maintenance_mva_harvard: u8 @ 0..=3,
    }
}
impl Display for MemoryModelFeature1 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tBTB: {}\n\
            \tCache operations (both):\n\
            \t\tTest and clean: {}\n\
            \tCache operations (unified):\n\
            \t\tL1 - All maintenance operations: {}\n\
            \t\tL1 - Cache line maintenance (Set/Way): {}\n\
            \t\tL1 - Cache line maintenance (MVA): {}\n\
            \tCache Operations (Harvard):\n\
            \t\tL1 - All maintenance operations: {}\n\
            \t\tL1 - Cache line maintenance (Set/Way): {}\n\
            \t\tL1 - Cache line maintenance (MVA): {}\n",
            match self.btb() {
                1 => "supported (required: BP flush on VA change)",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.dcache_test_and_clean_ops() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.l1_cache_all_maintenance_unified() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.l1_cache_line_maintenance_set_way_unified() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.l1_cache_line_maintenance_mva_unified() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.l1_cache_all_maintenance_harvard() {
                3 =>
                    "invalidate:icache+BP,dcache,both  clean:dcache+recursive+dirty clean+invalidate:dcache+recursive+dirty",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.l1_cache_line_maintenance_set_way_harvard() {
                3 => "clean,clean+invalidate:dcacheline invalidate:dcacheline,icacheline",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.l1_cache_line_maintenance_mva_harvard() {
                2 => "clean,clean+invalidate:dcacheline invalidate:dcacheline,icacheline,BTB",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct MemoryModelFeature2(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        hardware_access_flag: u8 @ 28..=31,
        wait_for_int_stalling: u8 @ 24..=27,
        memory_barriers: u8 @ 20..=23,
        tlb_maintenance_unified: u8 @ 16..=19,
        tlb_maintenance_harvard: u8 @ 12..=15,
        cache_maintenance_range_harvard: u8 @ 8..=11,
        background_prefetch_cache_range_harvard: u8 @ 4..=7,
        foreground_prefetch_cache_range_harvard: u8 @ 0..=3,
    }
}
impl Display for MemoryModelFeature2 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tHardware access flag: {}\n\
            \tWait For Interrupt (WFI) stalling: {}\n\
            \tMemory barriers: {}\n\
            \tTLB maintenance operations (unified): {}\n\
            \tTLB maintenance operations (Harvard): {}\n\
            \tCache maintenance range operations (Harvard): {}\n\
            \tBackground prefetch cache range operations (Harvard): {}\n\
            \tForeground prefetch cache range operations (Harvard): {}\n",
            match self.hardware_access_flag() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.wait_for_int_stalling() {
                1 => "supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.memory_barriers() {
                2 => "DataSynchronizationBarrier,PrefetchFlush,DataMemoryBarrier",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.tlb_maintenance_unified() {
                2 => "invalidate:all entries,by MVA,by ASID match",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.tlb_maintenance_harvard() {
                2 => "invalidate TLB:i,d,i+d x all,MVA,ASID",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.cache_maintenance_range_harvard() {
                1 => "clean,clean+invalidate range:dcache invalidate range:icache,dcache",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.background_prefetch_cache_range_harvard() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.foreground_prefetch_cache_range_harvard() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct MemoryModelFeature3(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        hierarchical_cache_maintenance_mva: u8 @ 4..=7,
        hierarchical_cache_maintenance_set_way: u8 @ 0..=3,
    }
}
impl Display for MemoryModelFeature3 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tHierarchical cache maintenance (by MVA): {}\n\
            \tHierarchical cache maintenance (by Set/Way): {}\n",
            match self.hierarchical_cache_maintenance_mva() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.hierarchical_cache_maintenance_set_way() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct IsaFeature0(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        divide: u8 @ 24..=27,
        debug: u8 @ 20..=23,
        coprocessor: u8 @ 16..=19,
        combined_compare_and_branch: u8 @ 12..=15,
        bitfield:  u8 @ 8..=11,
        bit_counting: u8 @ 4..=7,
        atomic_load_store: u8 @ 0..=3,
    }
}
impl Display for IsaFeature0 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tDivide instructions: {}\n\
            \tDebug instructions: {}\n\
            \tCoprocessor instructions: {}\n\
            \tCombined compare-and-branch instructions: {}\n\
            \tBitfield instructions: {}\n\
            \tBit counting instructions: {}\n\
            \tAtomic load and store instructions: {}\n",
            match self.divide() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.debug() {
                1 => "BKPT",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.coprocessor() {
                4 => "CDP,LDC,MCR,MRC,STC,MCRR,MRRC [2]",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.combined_compare_and_branch() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.bitfield() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.bit_counting() {
                1 => "CLZ",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.atomic_load_store() {
                1 => "SWP,SWPB",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct IsaFeature1(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        java: u8 @ 28..=31,
        interworking: u8 @ 24..=27,
        immediate: u8 @ 20..=23,
        if_then: u8 @  16..=19,
        sign_zero_extend: u8 @ 12..=15,
        exception_2: u8 @ 8..=11,
        exception_1: u8 @ 4..=7,
        endianness_ctrl: u8 @ 0..=3,
    }
}
impl Display for IsaFeature1 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tJava instructions: {}\n\
            \tInterworking instructions: {}\n\
            \tImmediate instructions: {}\n\
            \tIf-then instructions: {}\n\
            \tSign/Zero extending instructions: {}\n\
            \tException 2 instructions: {}\n\
            \tException 1 instructions: {}\n\
            \tEndianness control instructions: {}\n",
            match self.java() {
                1 => "BXJ,J bit in PSRs",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.interworking() {
                2 => "BX,T bit in PSRs, BLX and PC loads have BX behaviour",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.immediate() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.if_then() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.sign_zero_extend() {
                2 => "SXT/UXT [A] B/B16/H",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.exception_2() {
                1 => "SRS,RFE,CPS",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.exception_1() {
                1 => "LDM^,LDM(pc)^,STM^",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.endianness_ctrl() {
                1 => "SETEND,E bit in PSRs",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct IsaFeature2(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        reversal: u8 @ 28..=31,
        psr: u8 @ 24..=27,
        adv_umult: u8 @ 20..=23,
        adv_smult: u8 @ 16..=19,
        mult: u8 @ 12..=15,
        multi_access_interruptible: u8 @ 8..=11,
        memory_hint: u8 @ 4..=7,
        load_store:  u8 @ 0..=3,
    }
}
impl Display for IsaFeature2 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tReversal instructions: {}\n\
            \tPSR instructions: {}\n\
            \tAdvanced unsigned multiply instructions: {}\n\
            \tAdvanced signed multiply instructions: {}\n\
            \tMultiply instructions: {}\n\
            \tMulti-access interruptible instructions: {}\n\
            \tMemory hint instructions: {}\n\
            \tLoad/store instructions: {}\n",
            match self.reversal() {
                1 => "REV,REV16,REVSH",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            // MRS/MSR exception return instructions for data-processing
            match self.psr() {
                1 => "MRS,MSR",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.adv_umult() {
                2 => "UMULL,UMLAL,UMAAL",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.adv_smult() {
                3 =>
                    "SMULL/SMLAL SMLA[L](B|T|W)(B|T|W),Q bit in PSRs, SML[A|S][L](X|D)[X|D] SMML(A|S)[R] SMMUL[R] SMU(A|S)D[X]",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.mult() {
                1 => "MLA",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.multi_access_interruptible() {
                1 => "restartable LDM,STM",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.memory_hint() {
                2 => "PLD",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.load_store() {
                1 => "LDRD,STRD",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct IsaFeature3(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        thumb2: u8 @ 28..=31,
        true_nop: u8 @ 24..=27,
        thumb_copy: u8 @ 20..=23,
        table_branch: u8 @ 16..=19,
        synchronization: u8 @ 12..=15,
        svc: u8 @ 8..=11,
        simd: u8 @ 4..=7,
        saturate: u8 @ 0..=3,
    }
}
impl Display for IsaFeature3 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tThumb-2 extensions: {}\n\
            \tTrue NOP instructions: {}\n\
            \tThumb copy instructions: {}\n\
            \tTable branch instructions: {}\n\
            \tSynchronization primitive instructions: {}\n\
            \tSVC instructions: {}\n\
            \tSIMD instructions: {}\n\
            \tSaturate instructions: {}\n",
            match self.thumb2() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.true_nop() {
                1 => "NOP (+capability: additional NOP-compatible hints); NOP16 not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.thumb_copy() {
                1 => "MOV (3) low-reg -> low-reg, CPY alias",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.table_branch() {
                0 => "not supported",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.synchronization() {
                2 => "LDREX[B|H|D] STREX[B|H|D] CLREX",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.svc() {
                1 => "SVC",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.simd() {
                3 =>
                    "PKH[B/T] (U[H|Q]|S[H]|Q)ADD(8|16|SUBX) (U[H|Q]|S[H]|Q)SUB(8|16|ADDX) SEL SSAT[16] (U|S)XT[A]B16 USAD[A]8 USAT(8|16), GE[3:0] bits in PSRs",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.saturate() {
                1 => "QADD,QDADD,QDSUB,QSUB,Q bit in PSRs",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            }
        )
    }
}

proc_bitfield::bitfield! {
    #[derive(Copy, Clone)]
    pub struct IsaFeature4(u32): Debug, FromStorage, IntoStorage, DerefStorage {
        fractional_synchronization: u8 @ 20..=23,
        barrier: u8 @ 16..=19,
        smc: u8 @ 12..=15,
        writeback: u8 @ 8..=11,
        shift: u8 @ 4..=7,
        unprivileged: u8 @ 0..=3,
    }
}
impl Display for IsaFeature4 {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "\tFractional support for synchronization primitive instructions: {}\n\
            \tBarrier instructions: {}\n\
            \tSMC instructions: {}\n\
            \tWriteback instructions: {}\n\
            \tWith shift instructions: {}\n\
            \tUnprivileged instructions: {}\n",
            match self.fractional_synchronization() {
                0 => "none",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.barrier() {
                0 => "CP15 only",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.smc() {
                1 => "SMC",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.writeback() {
                1 => "all defined writeback addressing modes",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.shift() {
                4 =>
                    "shifts of loads and stores over LSL0-3, constant/register controlled shift options",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
            match self.unprivileged() {
                1 => "LDRBT,LDRT,STRBT,STRT",
                _ => "\x1b[31mUNKNOWN\x1b[0m",
            },
        )
    }
}
