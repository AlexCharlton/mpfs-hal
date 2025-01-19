#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::digital::Wait;
use mpfs_hal::gpio::*;
use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut p8_3 = Output::new(P8_3::take().unwrap());
    let mut input = Input::new(P8_45::take().unwrap());

    loop {
        Timer::after_millis(700).await;
        p8_3.set_high().unwrap();
        Timer::after_millis(700).await;
        p8_3.set_low().unwrap();
        input.wait_for_any_edge().await.unwrap();
        log::info!("Interrupt received");
    }
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::gpio::init();
    mpfs_hal::init_logger(log::LevelFilter::Info);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
