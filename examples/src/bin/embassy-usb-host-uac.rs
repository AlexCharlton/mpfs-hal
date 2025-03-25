#![no_std]
#![no_main]

use embassy_usb::handlers::uac::UacHandler;
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::host::{DeviceEvent::Connected, UsbHostDriver};
use mpfs_hal::Peripheral;
use mpfs_hal_embassy::usb::host::UsbHost;

#[macro_use]
extern crate mpfs_hal;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello world!");

    let mut usbhost = UsbHost::take().unwrap();
    usbhost.start();

    println!("Detecting device");
    // Wait for root-port to detect device
    let speed = loop {
        match usbhost.wait_for_device_event().await {
            Connected(speed) => break speed,
            _ => {}
        }
    };

    println!("Found device with speed = {:?}", speed);
    let enum_info = usbhost.enumerate_root(speed, 1).await.unwrap();

    if let Ok(mut uac) = UacHandler::try_register(&usbhost, enum_info, speed).await {
        println!("UAC registered");
        let sampling_freq = uac
            .get_sampling_freq(uac.input_terminal().clock_source_id())
            .await;
        println!("[UAC] Current Sampling Frequency: {:?}", sampling_freq);

        let lang_id = uac.get_supported_language().await;
        println!("[UAC] Supported Language: {:?}", lang_id);

        let terminal_name = uac
            .get_string(uac.input_terminal().terminal_name(), lang_id.unwrap())
            .await;
        println!("[UAC] Terminal Name: {:?}", terminal_name);

        const SAMPLE_RATE: f32 = 44100.0;
        const FREQUENCY: f32 = 440.0;
        const CHANNELS: usize = 16;
        const BYTES_PER_SAMPLE: usize = 4; // 24-bit audio in 32-bit container
        const FRAME_SIZE: usize = CHANNELS * BYTES_PER_SAMPLE;
        static mut PHASE: u32 = 0;

        uac.output_stream(|buffer: &mut [u8]| {
            // Generates a 440Hz sine wave

            // Calculate number of frames in this buffer
            let frame_count = buffer.len() / FRAME_SIZE;

            for frame in 0..frame_count {
                // Get and increment phase
                let phase =
                    unsafe { PHASE } as f32 / (1 << 30) as f32 * 2.0 * core::f32::consts::PI;
                let phase_inc = (FREQUENCY * 2.0 * core::f32::consts::PI / SAMPLE_RATE
                    * (1 << 30) as f32) as u32;
                unsafe { PHASE = PHASE.wrapping_add(phase_inc) };

                // Calculate sine wave sample (-1.0 to 1.0)
                let sample = libm::sinf(phase);

                // Convert to 24-bit PCM (signed, range -8388608 to 8388607)
                let pcm_value = (sample * 8388607.0) as i32;

                // Write to first channel only
                let offset = frame * FRAME_SIZE;
                buffer[offset] = 0; // Least significant byte is unused
                buffer[offset + 1] = (pcm_value & 0xFF) as u8;
                buffer[offset + 2] = ((pcm_value >> 8) & 0xFF) as u8;
                buffer[offset + 3] = ((pcm_value >> 16) & 0xFF) as u8;
            }
        })
        .await
        .unwrap();
    } else {
        println!("Failed to register UAC");
    }
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
