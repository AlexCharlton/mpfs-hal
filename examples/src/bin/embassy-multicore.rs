#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use alloc::vec::Vec;
use embassy_time::{Instant, Timer};
use mpfs_hal::hart_id;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello World from Rust from hart {}!", hart_id());

    let mut xs = Vec::new();
    xs.push(1);
    println!("Got value {} from the heap!", xs.pop().unwrap());

    let now = Instant::now();
    loop {
        let elapsed = Instant::now() - now;
        println!("{} ms", elapsed.as_millis());
        Timer::after_millis(1000).await;
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    println!("Hart {} woke up!", hart_id());
    Timer::after_millis(1500).await;
    println!("Hart {} again at {}", hart_id(), Instant::now());
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(_spawner: embassy_executor::Spawner) {
    println!("Hart {} woke up!", hart_id());
}

#[mpfs_hal::init_once]
fn init_once() {
    println!("This gets run before (almost) anything else...");
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
