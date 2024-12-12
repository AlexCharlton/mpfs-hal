#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

#[mpfs_hal::hart1_main]
pub fn hart1_main() {
    println!("Hello World from Rust from hart 1!")
}

#[mpfs_hal::hart2_main]
pub fn hart2_main() {
    println!("Hello World from Rust from hart 2!")
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
