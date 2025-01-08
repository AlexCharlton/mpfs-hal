#![no_std]

use core::ptr::addr_of_mut;

mod bindings;
pub use bindings::*;

mod clint;
pub use clint::*;

mod encoding;
pub use encoding::*;

mod gpio;
pub use gpio::*;

mod sysreg;
pub use sysreg::*;

mod timer;
pub use timer::*;

mod uart;
pub use uart::*;

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
    0x1000000000 + 0x80000000
}
