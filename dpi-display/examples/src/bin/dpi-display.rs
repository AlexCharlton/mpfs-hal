#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use dpi_display::*;
use mpfs_hal::{pac, Peripheral};

fn generate_test_data() -> Vec<u8> {
    let mut test_data: Vec<u8> = vec![0; BUFFER_SIZE * 2];
    let mut i = 0;
    // write 2 buffers worth of data
    while i < 144000 * 2 {
        if i < 72000 {
            // RRRR GGGG BBBB WWWW
            test_data[i * 8..i * 8 + 48].copy_from_slice(&[
                0xFF, 0x00, 0x00, // R
                0xFF, 0x00, 0x00, // R
                0xFF, 0x00, 0x00, // R
                0xFF, 0x00, 0x00, // R
                0x00, 0xFF, 0x00, // G
                0x00, 0xFF, 0x00, // G
                0x00, 0xFF, 0x00, // G
                0x00, 0xFF, 0x00, // G
                0x00, 0x00, 0xFF, // B
                0x00, 0x00, 0xFF, // B
                0x00, 0x00, 0xFF, // B
                0x00, 0x00, 0xFF, // B
                0xFF, 0xFF, 0xFF, // W
                0xFF, 0xFF, 0xFF, // W
                0xFF, 0xFF, 0xFF, // W
                0xFF, 0xFF, 0xFF, // W
            ]);
        } else if i < 144000 {
            // Just Red
            test_data[i * 8..i * 8 + 48].copy_from_slice(&[
                0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00,
                0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF,
                0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00,
                0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00,
            ]);
        } else if i < 216000 {
            // Just Green
            test_data[i * 8..i * 8 + 48].copy_from_slice(&[
                0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF,
                0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00,
                0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00,
                0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00,
            ]);
        } else {
            // Just Blue
            test_data[i * 8..i * 8 + 48].copy_from_slice(&[
                0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00,
                0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00,
                0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF,
                0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF,
            ]);
        }
        i += 6;
    }
    test_data
}

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");
    log::info!("Display dimensions: {}x{}", WIDTH, HEIGHT);
    log::info!("Buffer address: 0x{:x?}", pac::heap_end());

    let test_data = generate_test_data();
    let mut display = Display::take().unwrap();
    let mut line_offset = 0;

    loop {
        let mut buffer = display.get_buffer().await;
        let bytes_per_line = WIDTH * 3;

        // Calculate starting position in test data
        let start_pos = (line_offset * bytes_per_line) as usize;

        if start_pos + BUFFER_SIZE <= test_data.len() {
            // If we can copy in one go
            buffer.as_slice()[..].copy_from_slice(&test_data[start_pos..start_pos + BUFFER_SIZE]);
        } else {
            // Need to wrap around - do it in two copies
            let first_part = test_data.len() - start_pos;
            buffer.as_slice()[..first_part].copy_from_slice(&test_data[start_pos..]);
            buffer.as_slice()[first_part..].copy_from_slice(&test_data[..BUFFER_SIZE - first_part]);
        }

        line_offset = (line_offset + 1) % (test_data.len() / bytes_per_line);
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
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
