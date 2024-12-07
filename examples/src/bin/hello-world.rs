#![no_std]
#![no_main]

use mpfs_hal;

#[no_mangle]
pub fn __hart1_entry() {
    mpfs_hal::uart_puts(b"Hello World from Rust from hart 1!\n\0".as_ptr());
}

#[no_mangle]
pub fn __hart2_entry() {
    mpfs_hal::uart_puts(b"Hello World from Rust from hart 2!\n\0".as_ptr());
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::uart_print_panic(info);
    loop {}
}
