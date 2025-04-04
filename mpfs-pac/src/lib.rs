#![no_std]

mod bindings;
pub use bindings::*;

mod clint;
pub use clint::*;

mod encoding;
pub use encoding::*;

mod ethernet;
pub use ethernet::*;
mod gpio;
pub use gpio::*;

mod sysreg;
pub use sysreg::*;

mod timer;
pub use timer::*;

mod uart;
pub use uart::*;

mod usb;
pub use usb::*;

pub mod spi;
pub use spi::{QSPI, SPI0, SPI1};

#[inline]
pub fn hart_id() -> usize {
    let mut hart_id: usize;
    unsafe {
        core::arch::asm!("csrr {}, mhartid", out(reg) hart_id);
    }
    hart_id
}

pub fn last_linked_address() -> usize {
    &raw mut __app_hart_common_end as usize
}

pub fn last_address() -> usize {
    let base_address = if cfg!(feature = "upper-memory-layout") {
        0x10_0000_0000
    } else {
        0x8000_0000
    };
    let size_ram = if cfg!(feature = "beaglev-fire") {
        // 2GB
        0x8000_0000
    } else {
        panic!("Unsupported board: No RAM size defined");
    };

    base_address + size_ram
}
