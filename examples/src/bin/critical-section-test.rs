#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

use core::cell::RefCell;

use embassy_sync::{
    blocking_mutex::Mutex, blocking_mutex::raw::CriticalSectionRawMutex, once_lock::OnceLock,
};

fn counter() -> &'static Mutex<CriticalSectionRawMutex, RefCell<usize>> {
    static COUNTER: OnceLock<Mutex<CriticalSectionRawMutex, RefCell<usize>>> = OnceLock::new();
    COUNTER.get_or_init(|| Mutex::new(RefCell::new(0)))
}

#[mpfs_hal::hart1_main]
pub fn hart1_main() {
    println!("Hello World from Rust from hart 1!");
    let counter = counter();
    unsafe {
        println!(
            "Counter init from hart 1: {}",
            counter.lock(|c| *c.borrow())
        );
        for _ in 0..1000000 {
            counter.lock_mut(|c| *c.borrow_mut() += 1);
        }
        println!(
            "Counter final from hart 1: {}",
            counter.lock(|c| *c.borrow())
        );
    }
}

#[mpfs_hal::hart2_main]
pub fn hart2_main() {
    println!("Hello World from Rust from hart 2!");
    let counter = counter();
    unsafe {
        println!(
            "Counter init from hart 2: {}",
            counter.lock(|c| *c.borrow())
        );
        for _ in 0..1000000 {
            counter.lock_mut(|c| *c.borrow_mut() += 1);
        }
        println!(
            "Counter final from hart 2: {}",
            counter.lock(|c| *c.borrow())
        );
    }
}

#[mpfs_hal::hart3_main]
pub fn hart3_main() {
    println!("Hello World from Rust from hart 3!");
    let counter = counter();
    unsafe {
        println!(
            "Counter init from hart 3: {}",
            counter.lock(|c| *c.borrow())
        );
        for _ in 0..1000000 {
            counter.lock_mut(|c| *c.borrow_mut() += 1);
        }
        println!(
            "Counter final from hart 3: {}",
            counter.lock(|c| *c.borrow())
        );
    }
}

#[mpfs_hal::hart4_main]
pub fn hart4_main() {
    println!("Hello World from Rust from hart 4!");
    let counter = counter();
    unsafe {
        println!(
            "Counter init from hart 4: {}",
            counter.lock(|c| *c.borrow())
        );
        for _ in 0..1000000 {
            counter.lock_mut(|c| *c.borrow_mut() += 1);
        }
        println!(
            "Counter final from hart 4: {}",
            counter.lock(|c| *c.borrow())
        );
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}
