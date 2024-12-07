#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

use alloc::{format, vec::Vec};
use embassy_time::{Instant, Timer};
use mpfs_hal::{hart_id, uart_puts};

// TODO remove me
use mpfs_hal_embassy::Executor;
use static_cell::StaticCell;

#[no_mangle]
fn __hart1_entry() {
    static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
    EXECUTOR1.init(Executor::new()).run(|spawner| {
        spawner.must_spawn(hart1_task(spawner));
    });
}

#[embassy_executor::task]
async fn hart1_task(_spawner: embassy_executor::Spawner) {
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

#[no_mangle]
fn __hart2_entry() {
    static EXECUTOR2: StaticCell<Executor> = StaticCell::new();
    EXECUTOR2.init(Executor::new()).run(|spawner| {
        spawner.must_spawn(hart2_task(spawner));
    });
}

#[embassy_executor::task]
async fn hart2_task(_spawner: embassy_executor::Spawner) {
    let msg = format!("Hart {} woke up!\n\0", hart_id());
    uart_puts(msg.as_ptr());
    Timer::after_millis(1500).await;
    let msg = format!("Hart {} again at {}\n\0", hart_id(), Instant::now());
    uart_puts(msg.as_ptr());
}

#[no_mangle]
fn __hart3_entry() {
    static EXECUTOR3: StaticCell<Executor> = StaticCell::new();
    EXECUTOR3.init(Executor::new()).run(|spawner| {
        spawner.must_spawn(hart3_task(spawner));
    });
}

#[embassy_executor::task]
async fn hart3_task(_spawner: embassy_executor::Spawner) {
    let msg = format!("Hart {} woke up!\n\0", hart_id());
    uart_puts(msg.as_ptr());
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::uart_print_panic(info);
    loop {}
}
