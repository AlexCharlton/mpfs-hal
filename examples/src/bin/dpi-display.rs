#![no_std]
#![no_main]

use dpi_display::*;
use mpfs_hal::pac;

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

    // (*pac::GPIO2_LO).GPIO_IN & (1 << 26) != 0 // Buffer 0 unlocked
    // (*pac::GPIO2_LO).GPIO_IN & (1 << 27) != 0 // Buffer 1 locked
    loop {
        unsafe {
            pac::MSS_GPIO_set_output(pac::GPIO2_LO, 26, 1); // Buffer 0 ready
            pac::MSS_GPIO_set_output(pac::GPIO2_LO, 27, 0); // Buffer 1 not ready
            embassy_time::Timer::after_secs(1).await;
            pac::MSS_GPIO_set_output(pac::GPIO2_LO, 26, 0); // Buffer 0 not ready
            pac::MSS_GPIO_set_output(pac::GPIO2_LO, 27, 1); // Buffer 1 ready
            embassy_time::Timer::after_secs(1).await;
        }
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Debug);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}
