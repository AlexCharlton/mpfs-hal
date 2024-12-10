#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use embassy_time::{Instant, Timer};
use embedded_io::Write;
use mpfs_hal::{hart_id, uart};

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut uart = uart::Uart::new(uart::Peripheral::Uart0, uart::UartConfig::default());
    let now = Instant::now();
    uart.write_all(b"\n").unwrap();
    uart.write_fmt(format_args!(
        "Hello World from Rust from hart {}!\n",
        hart_id()
    ))
    .unwrap();

    let mut xs = Vec::new();
    xs.push(1);

    uart.write_fmt(format_args!(
        "Got value {} from the heap!\n",
        xs.pop().unwrap()
    ))
    .unwrap();

    loop {
        let elapsed = Instant::now() - now;
        uart.write_fmt(format_args!("{} ms\n", elapsed.as_millis()))
            .unwrap();
        Timer::after_millis(1000).await;
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    let mut uart = uart::Uart::new(uart::Peripheral::Uart0, uart::UartConfig::default());
    uart.write_fmt(format_args!("Hart {} woke up!\n", hart_id()))
        .unwrap();
    Timer::after_millis(1500).await;
    uart.write_fmt(format_args!(
        "Hart {} again at {}\n",
        hart_id(),
        Instant::now()
    ))
    .unwrap();
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(_spawner: embassy_executor::Spawner) {
    let mut uart = uart::Uart::new(uart::Peripheral::Uart0, uart::UartConfig::default());
    uart.write_fmt(format_args!("Hart {} woke up!\n", hart_id()))
        .unwrap();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::uart_print_panic(info);
    loop {}
}
