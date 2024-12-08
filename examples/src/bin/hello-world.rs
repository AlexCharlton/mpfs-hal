#![no_std]
#![no_main]

use mpfs_hal;

#[mpfs_hal::hart1_main]
pub fn hart1_main() {
    mpfs_hal::uart_puts(b"Hello World from Rust from hart 1!\n\0".as_ptr());
}

#[mpfs_hal::hart2_main]
pub fn hart2_main() {
    mpfs_hal::uart_puts(b"Hello World from Rust from hart 2!\n\0".as_ptr());
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::uart_print_panic(info);
    loop {}
}
