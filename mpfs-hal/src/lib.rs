#![no_std]

#[cfg(feature = "alloc")]
mod alloc;
#[cfg(feature = "alloc")]
use alloc::init_heap;

mod critical_section_impl;
critical_section::set_impl!(critical_section_impl::MPFSCriticalSection);

pub use mpfs_hal_procmacros::{hart1_main, hart2_main, hart3_main, hart4_main, init_once};

mod mutex;
pub use mutex::Mutex;

pub use mpfs_pac as pac;
pub use pac::hart_id;

mod peripheral;
pub use peripheral::*;

#[cfg(feature = "print")]
mod print;
#[cfg(feature = "print")]
pub use print::*;

#[cfg(feature = "log")]
mod logger;
#[cfg(feature = "log")]
pub use logger::init_logger;

pub mod ethernet;
pub mod gpio;
pub mod qspi;
pub mod uart;
pub mod usb;

//----------------------------------------------------------
// Entry points

extern "C" {
    fn __init_once();
    fn __init_once_embassy();
    fn __hart1_entry();
    fn __hart2_entry();
    fn __hart3_entry();
    fn __hart4_entry();
}

fn init_once() {
    unsafe {
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_CFM,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );

        #[cfg(feature = "alloc")]
        init_heap();
        #[cfg(feature = "print")]
        init_print();

        __init_once_embassy();
        __init_once();
    }
}

#[no_mangle]
extern "C" fn u54_1() {
    unsafe {
        // Rest of hardware initialization
        pac::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const pac::MIP_MSIP, options(nomem, nostack));

        pac::PLIC_init();
        pac::__enable_irq();
        // All other harts are put into wfi when they boot, so we can init_once from here
        init_once();

        // Now we wake up the other harts
        pac::raise_soft_interrupt(2);
        pac::raise_soft_interrupt(3);
        pac::raise_soft_interrupt(4);

        __hart1_entry();
    }
}

#[no_mangle]
extern "C" fn u54_2() {
    unsafe {
        // Rest of hardware initialization
        pac::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const pac::MIP_MSIP, options(nomem, nostack));
        pac::PLIC_init();
        pac::__enable_irq();

        // Wait for the software interrupt
        core::arch::asm!("wfi", options(nomem, nostack));
        __hart2_entry();
    }
}

#[no_mangle]
extern "C" fn u54_3() {
    unsafe {
        // Rest of hardware initialization
        pac::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const pac::MIP_MSIP, options(nomem, nostack));
        pac::PLIC_init();
        pac::__enable_irq();

        // Wait for the software interrupt
        core::arch::asm!("wfi", options(nomem, nostack));
        __hart3_entry();
    }
}

#[no_mangle]
extern "C" fn u54_4() {
    unsafe {
        // Rest of hardware initialization
        pac::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const pac::MIP_MSIP, options(nomem, nostack));
        pac::PLIC_init();
        pac::__enable_irq();

        // Wait for the software interrupt
        core::arch::asm!("wfi", options(nomem, nostack));

        __hart4_entry();
    }
}
