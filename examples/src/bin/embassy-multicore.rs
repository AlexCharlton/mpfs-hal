#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

use alloc::{format, vec::Vec};
use embassy_time::{Instant, Timer};
use mpfs_hal::{hart_id, uart_puts};

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let now = Instant::now();
    uart_puts(b"\n\0".as_ptr());
    let msg = format!("Hello World from Rust from hart {}!\n\0", hart_id());
    uart_puts(msg.as_ptr());

    let mut xs = Vec::new();
    xs.push(1);

    let msg = format!("Got value {} from the heap!\n\0", xs.pop().unwrap());
    uart_puts(msg.as_ptr());

    loop {
        let elapsed = Instant::now() - now;
        let msg = format!("{} ms\n\0", elapsed.as_millis());
        uart_puts(msg.as_ptr());
        Timer::after_millis(1000).await;
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    let msg = format!("Hart {} woke up!\n\0", hart_id());
    uart_puts(msg.as_ptr());
    Timer::after_millis(1500).await;
    let msg = format!("Hart {} again at {}\n\0", hart_id(), Instant::now());
    uart_puts(msg.as_ptr());
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(_spawner: embassy_executor::Spawner) {
    let msg = format!("Hart {} woke up!\n\0", hart_id());
    uart_puts(msg.as_ptr());
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::uart_print_panic(info);
    loop {}
}
