#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use mpfs_hal::Peripheral;
use mpfs_hal::gpio::*;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut led0 = Led0::take().unwrap();
    let mut p9_23 = Output::new(P9_23::take().unwrap());

    loop {
        Timer::after_millis(700).await;
        led0.set_high().unwrap();
        p9_23.set_high().unwrap();
        Timer::after_millis(700).await;
        led0.set_low().unwrap();
        p9_23.set_low().unwrap();
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(_spawner: embassy_executor::Spawner) {
    let mut user_button = UserButton::take().unwrap();
    loop {
        user_button.wait_for_any_edge().await.unwrap();
        if user_button.is_high().unwrap() {
            log::info!("Button released");
        } else {
            log::info!("Button pressed");
        }
    }
}

#[mpfs_hal_embassy::embassy_hart4_main]
async fn hart4_main(_spawner: embassy_executor::Spawner) {
    let mut p9_25 = Input::new(P9_25::take().unwrap());
    loop {
        p9_25.wait_for_any_edge().await.unwrap();
        if p9_25.is_high().unwrap() {
            log::info!("P9_25 high");
        } else {
            log::info!("P9_25 low");
        }
    }
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Debug);
    mpfs_hal::gpio::init();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}
