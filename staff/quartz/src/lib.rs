#![allow(internal_features)]
#![feature(core_intrinsics)]
#![feature(array_ptr_get)]
#![feature(pointer_is_aligned_to)]
#![feature(array_try_map)]
#![feature(likely_unlikely)]
#![no_std]

pub mod arch;
pub mod device;
pub mod sync;
