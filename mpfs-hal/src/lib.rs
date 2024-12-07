#![no_std]

use core::panic::PanicInfo;
use core::ptr::addr_of_mut;
use mpfs_pac as sys;

pub use sys::hart_id;

mod critical_section_impl;
critical_section::set_impl!(critical_section_impl::MPFSCriticalSection);

#[cfg(feature = "alloc")]
mod alloc;
#[cfg(feature = "alloc")]
pub use alloc::init_heap;

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
        sys::mss_config_clk_rst(
            sys::mss_peripherals__MSS_PERIPH_MMUART0,
            sys::MPFS_HAL_FIRST_HART as u8,
            sys::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        sys::MSS_UART_init(
            addr_of_mut!(sys::g_mss_uart0_lo),
            sys::MSS_UART_115200_BAUD,
            sys::MSS_UART_DATA_8_BITS | sys::MSS_UART_NO_PARITY | sys::MSS_UART_ONE_STOP_BIT,
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
        sys::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const sys::MIP_MSIP, options(nomem, nostack));

        sys::PLIC_init();
        sys::__enable_irq();
        init_once();

        sys::raise_soft_interrupt(2);
        sys::raise_soft_interrupt(3);
        sys::raise_soft_interrupt(4);

        __hart1_entry();
    }
}

#[no_mangle]
extern "C" fn u54_2() {
    unsafe {
        // Rest of hardware initialization
        sys::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const sys::MIP_MSIP, options(nomem, nostack));
        sys::PLIC_init();
        sys::__enable_irq();

        // Wait for the software interrupt
        core::arch::asm!("wfi", options(nomem, nostack));
        __hart2_entry();
    }
}

#[no_mangle]
extern "C" fn u54_3() {
    unsafe {
        // Rest of hardware initialization
        sys::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const sys::MIP_MSIP, options(nomem, nostack));
        sys::PLIC_init();
        sys::__enable_irq();

        // Wait for the software interrupt
        core::arch::asm!("wfi", options(nomem, nostack));
        __hart3_entry();
    }
}

#[no_mangle]
extern "C" fn u54_4() {
    unsafe {
        // Rest of hardware initialization
        sys::clear_soft_interrupt();
        core::arch::asm!("csrs mie, {}", const sys::MIP_MSIP, options(nomem, nostack));
        sys::PLIC_init();
        sys::__enable_irq();

        // Wait for the software interrupt
        core::arch::asm!("wfi", options(nomem, nostack));

        __hart4_entry();
    }
}

//------------------------------------------------------------------------------------

pub fn uart_puts(s: *const u8) {
    critical_section::with(|_| {
        uart_puts_no_lock(s);
    });
}

pub fn uart_puts_no_lock(s: *const u8) {
    unsafe {
        sys::MSS_UART_polled_tx_string(addr_of_mut!(sys::g_mss_uart0_lo), s);
    }
}

pub fn uart_print_panic(panic: &PanicInfo<'_>) {
    // Print panic message if available
    if let Some(location) = panic.location() {
        // We shouldn't rely on alloc/critical section while panicking
        uart_puts_no_lock(b"\nPANIC at\0".as_ptr());
        uart_puts_no_lock(location.file().as_bytes().as_ptr());
        uart_puts_no_lock(b":\0".as_ptr());
        // Convert number to string
        let mut line_buf = [0u8; 10];
        let mut buf = itoa::Buffer::new();
        let num_str = buf.format(location.line());

        // Copy to our null-terminated buffer
        for (i, &byte) in num_str.as_bytes().iter().enumerate() {
            line_buf[i] = byte;
        }
        uart_puts_no_lock(line_buf.as_ptr());
    }
}
