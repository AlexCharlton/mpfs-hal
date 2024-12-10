#![no_std]
#![no_main]

use embedded_io::Write;
use mpfs_hal::uart;

#[mpfs_hal::hart1_main]
pub fn hart1_main() {
    let mut uart = uart::Uart::new(uart::Peripheral::Uart0, uart::UartConfig::default());
    uart.write_fmt(format_args!("Hello World from Rust from hart 1!\n"))
        .unwrap();
}

#[mpfs_hal::hart2_main]
pub fn hart2_main() {
    let mut uart = uart::Uart::new(uart::Peripheral::Uart0, uart::UartConfig::default());
    uart.write_fmt(format_args!("Hello World from Rust from hart 2!\n"))
        .unwrap();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::uart_print_panic(info);
    loop {}
}
