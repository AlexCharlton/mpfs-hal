#![no_std]
#![no_main]

use embassy_usb_host::class::uac::UacHandler;
use embassy_usb_host::{BusRoute, BusState, bus};
use mpfs_hal::Peripheral;
use mpfs_hal_embassy::usb::host;
use static_cell::StaticCell;

static USB_BUS_STATE: StaticCell<BusState> = StaticCell::new();

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");

    let bus_state = USB_BUS_STATE.init(BusState::new());
    let driver = host::UsbHostDriver::take().unwrap();
    driver.start();
    let (mut bus_ctl, bus) = bus(driver, bus_state);

    loop {
        log::debug!("Detecting device");
        let speed = bus_ctl.wait_for_connection().await;

        log::info!("Found device with speed = {:?}", speed);

        let mut config_buf = [0u8; 512];
        let (enum_info, _config_len) = match bus
            .enumerate(BusRoute::Direct(speed), &mut config_buf)
            .await
        {
            Ok(r) => r,
            Err(e) => {
                log::error!("Enumeration failed: {:?}", e);
                continue;
            }
        };

        if let Ok(mut uac) = UacHandler::try_register(&bus, enum_info).await {
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

            let mut out = uac.output().await.unwrap();
            out.output_stream(|| bus_ctl.controller().is_connected(), generate_sine_wave)
                .await
                .unwrap();
        } else {
            log::error!("Failed to register UAC");
        }
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
        // Wrap phase so that floating point precision degradation doesn't cause phase to drift
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
