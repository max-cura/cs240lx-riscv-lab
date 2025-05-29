#[cfg(feature = "bcm2835")]
pub mod bcm2835;

#[cfg(feature = "rp2350")]
pub mod rp2350;

pub mod timing;
pub mod uart;
