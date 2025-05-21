#![no_std]
#![no_main]

use dpi_display::*;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::digital::Wait;
use mpfs_hal::{pac, Peripheral};

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");
    log::info!("Display dimensions: {}x{}", WIDTH, HEIGHT);
    let addr = (pac::heap_end() + 0x4_0000_0000) as *mut u64;
    log::info!("Hello world! Address: {:x?}", addr);
    unsafe {
        let mut i = 0;
        // write 2 buffers worth of data
        while i < 144000 * 2 {
            if i < 72000 {
                // RRRR GGGG BBBB WWWW
                *(addr.add(i + 0)) = 0xFF0000_FF0000_FF00;
                *(addr.add(i + 1)) = 0x00FF000000FF0000;
                *(addr.add(i + 2)) = 0xFF0000FF0000FF00;
                *(addr.add(i + 3)) = 0x0000FF0000FF0000;
                *(addr.add(i + 4)) = 0xFF0000FFFFFFFFFF;
                *(addr.add(i + 5)) = 0xFFFFFFFFFFFFFFFF;
            } else if i < 144000 {
                // Just Red
                *(addr.add(i + 0)) = 0xFF0000FF0000FF00;
                *(addr.add(i + 1)) = 0x00FF0000FF0000FF;
                *(addr.add(i + 2)) = 0x0000FF0000FF0000;
                *(addr.add(i + 3)) = 0xFF0000FF0000FF00;
                *(addr.add(i + 4)) = 0x00FF0000FF0000FF;
                *(addr.add(i + 5)) = 0x0000FF0000FF0000;
            } else if i < 216000 {
                // Just Green
                *(addr.add(i + 0)) = 0x00FF0000FF0000FF;
                *(addr.add(i + 1)) = 0x0000FF0000FF0000;
                *(addr.add(i + 2)) = 0xFF0000FF0000FF00;
                *(addr.add(i + 3)) = 0x00FF0000FF0000FF;
                *(addr.add(i + 4)) = 0x0000FF0000FF0000;
                *(addr.add(i + 5)) = 0xFF0000FF0000FF00;
            } else {
                // Just Blue
                *(addr.add(i + 0)) = 0x0000FF0000FF0000;
                *(addr.add(i + 1)) = 0xFF0000FF0000FF00;
                *(addr.add(i + 2)) = 0x00FF0000FF0000FF;
                *(addr.add(i + 3)) = 0x0000FF0000FF0000;
                *(addr.add(i + 4)) = 0xFF0000FF0000FF00;
                *(addr.add(i + 5)) = 0x00FF0000FF0000FF;
            }
            i += 6;
        }
        log::info!("Hello world! Value: {:x?}", *addr);
    }

    let mut buffer0_ready = Buffer0Ready::take().unwrap();
    let mut buffer1_ready = Buffer1Ready::take().unwrap();

    loop {
        buffer0_ready.set_high().unwrap();
        buffer1_ready.set_low().unwrap();
        embassy_time::Timer::after_secs(1).await;
        buffer0_ready.set_low().unwrap();
        buffer1_ready.set_high().unwrap();
        embassy_time::Timer::after_secs(1).await;
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(_spawner: embassy_executor::Spawner) {
    let mut buffer0_locked = Buffer0Locked::take().unwrap();
    let mut buffer1_locked = Buffer1Locked::take().unwrap();
    loop {
        while buffer0_locked.is_low().unwrap() {
            core::hint::spin_loop();
        }
        log::info!("Buffer 0 locked");
        while buffer0_locked.is_high().unwrap() {
            core::hint::spin_loop();
        }
        log::info!("Buffer 0 unlocked");
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
