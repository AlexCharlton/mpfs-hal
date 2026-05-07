#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

use core::cell::RefCell;

use embassy_sync::{
    blocking_mutex::Mutex, blocking_mutex::raw::CriticalSectionRawMutex, once_lock::OnceLock,
};
use embassy_time::Timer;

fn counter() -> &'static Mutex<CriticalSectionRawMutex, RefCell<usize>> {
    static COUNTER: OnceLock<Mutex<CriticalSectionRawMutex, RefCell<usize>>> = OnceLock::new();
    COUNTER.get_or_init(|| Mutex::new(RefCell::new(0)))
}

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(spawner: embassy_executor::Spawner) {
    println!("Hello World from Rust from hart 1!");
    let counter = counter();
    println!(
        "Counter init from hart 1: {}",
        counter.lock(|c| *c.borrow())
    );

    spawner.spawn(counter_task(counter).unwrap());
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(spawner: embassy_executor::Spawner) {
    println!("Hello World from Rust from hart 2!");
    let counter = counter();
    println!(
        "Counter init from hart 2: {}",
        counter.lock(|c| *c.borrow())
    );

    spawner.spawn(counter_task(counter).unwrap());
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(spawner: embassy_executor::Spawner) {
    println!("Hello World from Rust from hart 3!");
    let counter = counter();
    println!(
        "Counter init from hart 3: {}",
        counter.lock(|c| *c.borrow())
    );
    spawner.spawn(counter_task(counter).unwrap());
}

#[mpfs_hal_embassy::embassy_hart4_main]
async fn hart4_main(spawner: embassy_executor::Spawner) {
    println!("Hello World from Rust from hart 4!");
    let counter = counter();
    println!(
        "Counter init from hart 4: {}",
        counter.lock(|c| *c.borrow())
    );
    spawner.spawn(counter_task(counter).unwrap());
}

#[embassy_executor::task(pool_size = 4)]
async fn counter_task(counter: &'static Mutex<CriticalSectionRawMutex, RefCell<usize>>) {
    for _ in 0..1000000 {
        counter.lock(|c| *c.borrow_mut() += 1);
        Timer::after_nanos(10).await;
    }
    println!(
        "Counter final from hart {}: {}",
        mpfs_hal::hart_id(),
        counter.lock(|c| *c.borrow())
    );
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}
