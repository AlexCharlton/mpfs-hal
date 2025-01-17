#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use mpfs_hal::gpio::*;
use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut p8_3 = Output::new(P8_3::take().unwrap());
    let mut p8_45 = Output::new(P8_45::take().unwrap());

    loop {
        Timer::after_millis(700).await;
        p8_3.set_high().unwrap();
        p8_45.set_high().unwrap();
        Timer::after_millis(700).await;
        p8_3.set_low().unwrap();
        p8_45.set_low().unwrap();
    }
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::gpio::init();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
