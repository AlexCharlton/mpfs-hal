#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use mpfs_hal::gpio::*;
use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut p8_3 = Output::new(P8_3::take().unwrap());
    let mut p9_23 = Output::new(P9_23::take().unwrap());

    loop {
        Timer::after_millis(700).await;
        p8_3.set_high().unwrap();
        p9_23.set_high().unwrap();
        Timer::after_millis(700).await;
        p8_3.set_low().unwrap();
        p9_23.set_low().unwrap();
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(_spawner: embassy_executor::Spawner) {
    let mut p9_25 = Input::new(P9_25::take().unwrap());
    loop {
        p9_25.wait_for_any_edge().await.unwrap();
        if p9_25.is_high().unwrap() {
            log::info!("Button pressed");
        } else {
            log::info!("Button released");
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
