// bitmap values:
//  0 if the child or any of its descendents is allocated
//  1 otherwise
// floating chain:
//  region is present if it is neither completely free or completely full
// this is sufficient to capture all necessary information
//
// allocate( kind ):
//  if parent( kind ) is Just parent_kind,
//      if the floating_list for parent_kind isn't empty
//          commit( floating_list.head , parent_kind , kind )
//      else
//          repeat with the parent's parent
// commit( pred , pred_kind , kind )
//  mark the most significant free bit on pred's bitmap as full
//  if this fills pred's bitmap
//      mark the corresponding bit in pred's parent's bitmap as full and remove pred's parent from its floating list
//      pop the floating list for pred_kind
//  if pred_kind == kind
//      as ptr
//  else
//      commit ( selected , pred_kind - 1, kind )
//! Naive implementation of a Physical Memory Manager for the ARM1176JZF-S.

#[allow(non_upper_case_globals)]
const KiB: usize = 1024;
#[allow(non_upper_case_globals)]
const MiB: usize = 1024 * KiB;
use core::fmt::{Debug, Formatter};
use core::mem::{MaybeUninit, size_of};
use core::ptr::NonNull;
use thiserror::Error;

struct RegionInfo {
    bitmap: u16,
    next: u16,
    prev: u16,
}
const CHAIN_END: u16 = 0xffff;
#[repr(C)]
struct PMMInit {
    floating_lists: MaybeUninit<[Option<usize>; 3]>,
    regions: MaybeUninit<[RegionInfo; 0x2220]>,
    supersection_mask: MaybeUninit<u32>,
    floating_counts: MaybeUninit<[usize; 3]>,
    allocated_counts: MaybeUninit<[usize; 4]>,
}
impl PMMInit {
    unsafe fn from_self_ptr(mut this_ptr: NonNull<Self>) -> &'static mut PMM {
        let this = unsafe { this_ptr.as_mut() };
        this.floating_lists.write([None; 3]);
        let regions_ptr = this.regions.as_mut_ptr();
        let regions_ptr = regions_ptr.as_mut_ptr();
        for i in 0..0x2220 {
            unsafe {
                regions_ptr.offset(i).write(RegionInfo {
                    bitmap: 0xffff,
                    next: CHAIN_END,
                    prev: CHAIN_END,
                })
            }
        }
        this.supersection_mask.write(0xffff_ffff);
        this.floating_counts.write([0; 3]);
        this.allocated_counts.write([0; 4]);
        unsafe { core::mem::transmute::<&mut PMMInit, &mut PMM>(this) }
    }
}

pub unsafe fn pmm_init_at(ptr: NonNull<PMM>) -> &'static mut PMM {
    unsafe {
        ptr.write_bytes(0, size_of::<PMM>());
        PMMInit::from_self_ptr(ptr.cast::<PMMInit>())
    }
}

#[repr(C)]
pub struct PMM {
    floating_lists: [Option<usize>; 3],
    regions: [RegionInfo; 0x2220], // 8192 x 6B
    supersection_mask: u32,

    floating_counts: [usize; 3],
    allocated_counts: [usize; 4],
}
impl Debug for PMM {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        writeln!(
            f,
            "LP floating list: {} starting @ {:?}",
            self.floating_counts[0], self.floating_lists[0]
        )?;
        writeln!(
            f,
            "S floating list: {} starting @ {:?}",
            self.floating_counts[1], self.floating_lists[1]
        )?;
        writeln!(
            f,
            "SS floating list: {} starting @ {:?}",
            self.floating_counts[2], self.floating_lists[2]
        )?;
        writeln!(f, "SP floating list: {}", self.allocated_counts[0])?;
        writeln!(f, "LP floating list: {}", self.allocated_counts[1])?;
        writeln!(f, "S floating list: {}", self.allocated_counts[2])?;
        writeln!(f, "SS floating list: {}", self.allocated_counts[3])?;
        writeln!(f, "SS mask: {:08x}", self.supersection_mask)
    }
}

impl PMM {
    /// Requirements: used_regions consists of sorted, non-overlapping ranges
    pub unsafe fn initialize_once(&mut self, used_regions: &[(*mut u8, *mut u8)]) {
        self.supersection_mask = 0xffff_ffff;
        for rmut in &mut self.regions {
            rmut.bitmap = 0xffff;
            rmut.prev = CHAIN_END;
            rmut.next = CHAIN_END;
        }

        let mut i = 0;
        'outer: for j in 0usize..0x2_0000 {
            let r = { (sp_ptr(j), sp_ptr(j + 1)) };
            if r.1 <= used_regions[i].0 {
                // haven't reach used_regions[i] yet
                continue;
            } else {
                // regions overlap
                self.allocated_counts[0] += 1;
                self.regions[lp_idx(j / 16)].bitmap &= !(1 << (j % 16));
                while used_regions[i].1 <= r.1 {
                    // while used_regions[i] ends on the current page
                    i += 1;
                    if i >= used_regions.len() {
                        break 'outer;
                    }
                }
            }
        }
        for i in 0usize..0x200 {
            for j in (i * 16)..(i * 16 + 16) {
                if self.regions[lp_idx(j)].bitmap != 0xffff {
                    self.allocated_counts[1] += 1;
                    self.regions[s_idx(i)].bitmap &= !(1 << (j % 16));
                    if self.regions[lp_idx(j)].bitmap != 0 {
                        self.float(lp_idx(j), RegionKind::LargePage);
                    }
                }
            }
        }
        for i in 0usize..0x20 {
            for j in (i * 16)..(i * 16 + 16) {
                if self.regions[s_idx(j)].bitmap != 0xffff {
                    self.allocated_counts[2] += 1;
                    self.regions[ss_idx(i)].bitmap &= !(1 << (j % 16));
                    if self.regions[s_idx(j)].bitmap != 0 {
                        self.float(s_idx(j), RegionKind::Section);
                    }
                }
            }
        }
        for i in 0usize..32 {
            if self.regions[ss_idx(i)].bitmap != 0xffff {
                self.allocated_counts[3] += 1;
                self.supersection_mask &= !(1 << i);
                // uart1_sendln_bl!("SS Mask: {:08x}", self.supersection_mask);
                if self.regions[ss_idx(i)].bitmap != 0 {
                    self.float(ss_idx(i), RegionKind::Supersection);
                }
            }
        }
    }

    pub fn stats(&self) -> ([usize; 3], [usize; 4]) {
        (self.floating_counts, self.allocated_counts)
    }

    pub fn deallocate_region(
        &mut self,
        region: *mut u8,
        region_kind: RegionKind,
    ) -> Result<(), PMMDeallocError> {
        let region_no = match region_kind {
            RegionKind::SmallPage => {
                assert!(region.is_aligned_to(0x1000));
                region as usize / 0x1000
            }
            RegionKind::LargePage => {
                assert!(region.is_aligned_to(0x1_0000));
                region as usize / 0x1_0000
            }
            RegionKind::Section => {
                assert!(region.is_aligned_to(0x10_0000));
                region as usize / 0x10_0000
            }
            RegionKind::Supersection => {
                assert!(region.is_aligned_to(0x100_0000));
                region as usize / 0x100_0000
            }
        };
        self.mark_free_recursive(region_no, region_kind);
        self.allocated_counts[match region_kind {
            RegionKind::SmallPage => 0,
            RegionKind::LargePage => 1,
            RegionKind::Section => 2,
            RegionKind::Supersection => 3,
        }] -= 1;
        Ok(())
    }

    fn mark_free_recursive(&mut self, region_no: usize, region_kind: RegionKind) {
        // find the parent, free the bit
        // if the parent goes partial from full, put it on the chain
        // if the parent goes empty, remove it from the chain, and mark_free_recursive on the parent
        let (parent_no, parent_idx, parent_kind) = match region_kind {
            RegionKind::SmallPage => {
                let parent_no = region_no / 16;
                (parent_no, lp_idx(parent_no), RegionKind::LargePage)
            }
            RegionKind::LargePage => {
                let parent_no = region_no / 16;
                (parent_no, s_idx(parent_no), RegionKind::Section)
            }
            RegionKind::Section => {
                let parent_no = region_no / 16;
                (parent_no, ss_idx(parent_no), RegionKind::Supersection)
            }
            RegionKind::Supersection => {
                // special handling:
                self.supersection_mask |= 1 << region_no;
                return;
            }
        };
        let bitmap_init = self.regions[parent_idx].bitmap;
        let bitmap_new = bitmap_init | (1 << (region_no % 16));
        self.regions[parent_idx].bitmap = bitmap_new;
        if bitmap_new == 0xffff {
            // un-chain
            self.unfloat(parent_idx, parent_kind);
            self.mark_free_recursive(parent_no, parent_kind)
        } else if bitmap_init == 0 {
            // chain
            self.float(parent_idx, parent_kind);
        } else {
            // no chain modifications necessary
        }
    }

    fn commit(
        &mut self,
        pred: usize,
        pred_kind: RegionKind,
        alloc_kind: RegionKind,
    ) -> Result<*mut u8, PMMAllocError> {
        // preconditions:
        //  floating_idx points to a floating RegionInfo
        //  curr_rergion_kind is the RegionKind of floating_idx
        //  region_kind is the RegionKind we're allocating for, and is NOT Supersection
        // if we split a supersection to get here, then that supersection is marked in the
        // supersections bitmap
        let mut bitmap = self.regions[pred].bitmap;
        let lz = bitmap.leading_zeros();
        bitmap &= !(0x8000 >> lz);
        self.regions[pred].bitmap = bitmap;
        if bitmap == 0 {
            // no children that are empty
            // remove this region from the floating chain
            self.unfloat(pred, pred_kind);
        }
        let child_no = (16 - lz) as usize;
        if pred_kind == RegionKind::LargePage {
            let lp_no = pred;
            assert!(lp_no < 0x2000);
            let sp_no = child_no + (lp_no * 16);
            if alloc_kind == RegionKind::SmallPage {
                Ok(sp_ptr(sp_no))
            } else {
                unreachable!()
            }
        } else if pred_kind == RegionKind::Section {
            let s_no = pred - 0x2000;
            assert!(s_no < 0x200);
            let lp_no = child_no + (s_no * 16);
            if alloc_kind == RegionKind::LargePage {
                Ok(lp_ptr(lp_no))
            } else {
                self.commit(lp_idx(lp_no), RegionKind::LargePage, alloc_kind)
            }
        } else if pred_kind == RegionKind::Supersection {
            let ss_no = pred - 0x2200;
            assert!(ss_no < 0x20);
            let s_no = child_no + (ss_no * 16);
            if alloc_kind == RegionKind::Section {
                Ok(s_ptr(s_no))
            } else {
                self.commit(s_idx(s_no), RegionKind::Section, alloc_kind)
            }
        } else {
            unreachable!()
        }
    }
    pub fn allocate_region(&mut self, region_kind: RegionKind) -> Result<*mut u8, PMMAllocError> {
        let mut curr_region_kind = region_kind;
        // okay, couldn't match from immediate chain
        loop {
            curr_region_kind = match curr_region_kind.next_smallest() {
                None => break,
                Some(x) => x,
            };
            if let Some(floating_idx) = self.floating_list(curr_region_kind).unwrap() {
                let ix = self.commit(floating_idx, curr_region_kind, region_kind);
                if ix.is_ok() {
                    self.allocated_counts[match region_kind {
                        RegionKind::SmallPage => 0,
                        RegionKind::LargePage => 1,
                        RegionKind::Section => 2,
                        RegionKind::Supersection => 3,
                    }] += 1;
                }
                return ix;
            }
        }
        // Okay, so there were no partial LPages, Sections, or Supersections. Thus, we must split a
        // supersection.
        // If supersection_mask has any bits nonzero, there is a free supersection.
        if self.supersection_mask != 0 {
            let lz = self.supersection_mask.leading_zeros();
            self.supersection_mask &= !(0x8000_0000 >> lz);
            let ss_no = 31 - lz;
            if region_kind == RegionKind::Supersection {
                // use it directly
                self.allocated_counts[3] += 1;
                Ok(ss_ptr(ss_no as usize))
            } else {
                // float the supersection
                let ss_idx = ss_idx(ss_no as usize);
                self.float(ss_idx, RegionKind::Supersection);
                let ix = self.commit(ss_idx, RegionKind::Supersection, region_kind);
                if ix.is_ok() {
                    self.allocated_counts[match region_kind {
                        RegionKind::SmallPage => 0,
                        RegionKind::LargePage => 1,
                        RegionKind::Section => 2,
                        RegionKind::Supersection => 3,
                    }] += 1;
                }
                return ix;
            }
        } else {
            return Err(PMMAllocError::OutOfMemory);
        }
    }
    fn unfloat(&mut self, idx: usize, kind: RegionKind) {
        let i = match kind {
            RegionKind::SmallPage => return,
            RegionKind::LargePage => 0,
            RegionKind::Section => 1,
            RegionKind::Supersection => 2,
        };
        self.floating_counts[i] -= 1;
        let next = self.regions[idx].next;
        if next != CHAIN_END {
            self.regions[next as usize].prev = CHAIN_END;
        }
        let prev = self.regions[idx].prev;
        if prev != CHAIN_END {
            self.regions[prev as usize].next = next;
        } else {
            self.floating_lists[i] = match next {
                CHAIN_END => None,
                x => Some(x as usize),
            };
        }
    }
    fn float(&mut self, idx: usize, kind: RegionKind) {
        let i = match kind {
            RegionKind::SmallPage => return,
            RegionKind::LargePage => 0,
            RegionKind::Section => 1,
            RegionKind::Supersection => 2,
        };
        self.floating_counts[i] += 1;
        self.regions[idx].prev = CHAIN_END;
        self.regions[idx].next = self.floating_lists[i]
            .map(|x| x as u16)
            .unwrap_or(CHAIN_END);
        self.floating_lists[i] = Some(idx);
    }
    fn floating_list(&self, region_kind: RegionKind) -> Option<Option<usize>> {
        match region_kind {
            RegionKind::LargePage => Some(self.floating_lists[0]),
            RegionKind::Section => Some(self.floating_lists[1]),
            RegionKind::Supersection => Some(self.floating_lists[2]),
            RegionKind::SmallPage => None,
        }
    }
    #[allow(dead_code)]
    fn floating_list_mut(&mut self, region_kind: RegionKind) -> Option<&mut Option<usize>> {
        match region_kind {
            RegionKind::LargePage => Some(&mut self.floating_lists[0]),
            RegionKind::Section => Some(&mut self.floating_lists[1]),
            RegionKind::Supersection => Some(&mut self.floating_lists[2]),
            RegionKind::SmallPage => None,
        }
    }
}

fn ss_ptr(ss_no: usize) -> *mut u8 {
    assert!(ss_no < 0x20);
    (ss_no * (16 * MiB)) as *mut u8
}
fn s_ptr(s_no: usize) -> *mut u8 {
    assert!(s_no < 0x200);
    (s_no * (1 * MiB)) as *mut u8
}
fn lp_ptr(lp_no: usize) -> *mut u8 {
    assert!(lp_no < 0x2000);
    (lp_no * (64 * KiB)) as *mut u8
}
fn sp_ptr(sp_no: usize) -> *mut u8 {
    assert!(sp_no < 0x20000);
    (sp_no * (4 * KiB)) as *mut u8
}

fn ss_idx(ss_no: usize) -> usize {
    assert!(ss_no < 0x20);
    ss_no + 0x2200
}
fn s_idx(s_no: usize) -> usize {
    assert!(s_no < 0x200);
    s_no + 0x2000
}
fn lp_idx(lp_no: usize) -> usize {
    assert!(lp_no < 0x2000);
    lp_no + 0
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum RegionKind {
    /// 4KB
    SmallPage,
    // 64KB
    LargePage,
    // 1M
    Section,
    // 16M
    Supersection,
}
impl RegionKind {
    fn next_smallest(self) -> Option<Self> {
        match self {
            RegionKind::SmallPage => Some(Self::LargePage),
            RegionKind::LargePage => Some(Self::Section),
            RegionKind::Section => Some(Self::Supersection),
            RegionKind::Supersection => None,
        }
    }
    pub fn size(&self) -> usize {
        match self {
            RegionKind::SmallPage => 4 * KiB,
            RegionKind::LargePage => 64 * KiB,
            RegionKind::Section => 1 * MiB,
            RegionKind::Supersection => 16 * MiB,
        }
    }
}

#[derive(Debug, Error)]
pub enum PMMAllocError {
    #[error(
        "Allocating physical region failed: no aligned contiguous free region of sufficient size"
    )]
    OutOfMemory,
}
#[derive(Debug, Error)]
pub enum PMMDeallocError {}
