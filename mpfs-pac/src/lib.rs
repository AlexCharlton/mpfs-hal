#![no_std]

use core::ptr::addr_of_mut;

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

mod qspi;
pub use qspi::*;

#[inline]
pub fn hart_id() -> usize {
    let mut hart_id: usize;
    unsafe {
        core::arch::asm!("csrr {}, mhartid", out(reg) hart_id);
    }
    hart_id
}

pub fn last_linked_address() -> usize {
    addr_of_mut!(__app_hart_common_end) as usize
}

pub fn last_address() -> usize {
    let base_address = if cfg!(feature = "upper-memory-layout") {
        0x10_0000_0000
    } else {
        0x8000_0000
    };
    let size_ram = if cfg!(feature = "beaglev-fire") {
        if cfg!(feature = "upper-memory-layout") {
            // 2GB
            0x8000_0000
        } else {
            // 1GB
            0x4000_0000
        }
    } else {
        panic!("Unsupported board: No RAM size defined");
    };

    base_address + size_ram
}
