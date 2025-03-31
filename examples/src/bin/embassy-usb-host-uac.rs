#![no_std]
#![no_main]

use embassy_usb::handlers::uac::UacHandler;
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::host::{DeviceEvent::Connected, UsbHostDriver};
use mpfs_hal::Peripheral;
use mpfs_hal_embassy::usb::host::UsbHost;

extern crate mpfs_hal;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");

    let mut usbhost = UsbHost::take().unwrap();
    usbhost.start();

    log::info!("Detecting device");
    // Wait for root-port to detect device
    let speed = loop {
        match usbhost.wait_for_device_event().await {
            Connected(speed) => break speed,
            _ => {}
        }
    };

    log::info!("Found device with speed = {:?}", speed);
    let enum_info = usbhost.enumerate_root(speed, 1).await.unwrap();

    if let Ok(mut uac) = UacHandler::try_register(&usbhost, enum_info, speed).await {
        log::info!("UAC registered");
        let sampling_freq = uac
            .get_sampling_freq(uac.input_terminal().clock_source_id())
            .await;
        log::info!("[UAC] Current Sampling Frequency: {:?}", sampling_freq);

        let lang_id = uac.get_supported_language().await;
        log::info!("[UAC] Supported Language: {:?}", lang_id);

        let terminal_name = uac
            .get_string(uac.input_terminal().terminal_name(), lang_id.unwrap())
            .await;
        log::info!("[UAC] Terminal Name: {:?}", terminal_name);

        uac.output_stream(generate_sine_wave).await.unwrap();
    } else {
        log::error!("Failed to register UAC");
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

static mut PHASE: f32 = 0.0;
fn generate_sine_wave(buffer: &mut [u8]) {
    const SAMPLE_RATE: f32 = 44100.0;
    const FREQUENCY: f32 = 440.0;
    const CHANNELS: usize = 16;
    const BYTES_PER_SAMPLE: usize = 4; // 24-bit audio in 32-bit container
    const FRAME_SIZE: usize = CHANNELS * BYTES_PER_SAMPLE;
    const TWO_PI: f32 = 2.0 * core::f32::consts::PI;
    let phase_inc = (FREQUENCY / SAMPLE_RATE) * TWO_PI;

    // Generates a 440Hz sine wave

    // Calculate number of frames in this buffer
    let frame_count = buffer.len() / FRAME_SIZE;

    for frame in 0..frame_count {
        // Get phase as normalized 0-1 value first
        let phase = unsafe { PHASE };

        // Calculate next phase increment
        unsafe { PHASE = phase + phase_inc };

        // Calculate sine wave sample (-1.0 to 1.0)
        let sample = libm::sinf(phase);

        // Apply slight amplitude safety margin to prevent clipping
        let pcm_value = (sample * 8388607.0 * 0.98) as i32;

        // Write sample in little-endian format
        let offset = frame * FRAME_SIZE;
        buffer[offset + 0] = 0; // Padding byte
        buffer[offset + 1] = pcm_value as u8;
        buffer[offset + 2] = (pcm_value >> 8) as u8;
        buffer[offset + 3] = (pcm_value >> 16) as u8;
    }

    unsafe {
        // Wrap phase so that floating point precision degredation doesn't cause phase to drift
        if PHASE > TWO_PI {
            PHASE = PHASE % TWO_PI;
        }
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
