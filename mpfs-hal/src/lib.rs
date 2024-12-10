#![no_std]

use core::panic::PanicInfo;
use core::ptr::addr_of_mut;

pub use mpfs_pac as pac;

pub use pac::hart_id;

mod critical_section_impl;
critical_section::set_impl!(critical_section_impl::MPFSCriticalSection);

#[cfg(feature = "alloc")]
mod alloc;
#[cfg(feature = "alloc")]
pub use alloc::init_heap;

pub use mpfs_hal_procmacros::{hart1_main, hart2_main, hart3_main, hart4_main};

pub mod uart;

//----------------------------------------------------------
// Entry points

extern "C" {
    fn __init_once();
    fn __hart1_entry();
    fn __hart2_entry();
    fn __hart3_entry();
    fn __hart4_entry();
}

// TODO: Make configurable
fn init_once() {
    unsafe {
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_MMUART0,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        pac::MSS_UART_init(
            addr_of_mut!(pac::g_mss_uart0_lo),
            pac::MSS_UART_115200_BAUD,
            pac::MSS_UART_DATA_8_BITS | pac::MSS_UART_NO_PARITY | pac::MSS_UART_ONE_STOP_BIT,
        );
        #[cfg(feature = "alloc")]
        init_heap();

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
        init_once();

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

//------------------------------------------------------------------------------------

pub fn uart_print_panic(panic: &PanicInfo<'_>) {
    use embedded_io::Write;
    use uart::*;

    // Print panic message if available
    if let Some(location) = panic.location() {
        // We shouldn't rely on alloc/critical section while panicking
        let mut uart = Uart::new(Peripheral::Uart0, UartConfig::default());
        uart.write_fmt(format_args!(
            "PANIC at {}:{}",
            location.file(),
            location.line()
        ))
        .unwrap();
    }
}
