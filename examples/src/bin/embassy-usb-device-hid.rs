#![no_std]
#![no_main]

// https://github.com/embassy-rs/embassy/blob/main/examples/stm32f4/src/bin/usb_hid_mouse.rs used as reference

use aligned::{Aligned, A4};
use embassy_futures::join::join;
use embassy_time::Timer;
use embassy_usb::class::hid::{HidWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::Builder;
use usbd_hid::descriptor::{MouseReport, SerializedDescriptor};

use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut driver = mpfs_hal_embassy::usb::device::UsbDriver::take().unwrap();
    driver.set_full_speed();
    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("HID mouse example");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors, and they should be 4-byte aligned.
    let mut config_descriptor = Aligned::<A4, _>([0; 256]);
    let mut bos_descriptor = Aligned::<A4, _>([0; 256]);
    let mut control_buf = Aligned::<A4, _>([0; 64]);

    let mut request_handler = MyRequestHandler {};

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor.as_mut_slice(),
        bos_descriptor.as_mut_slice(),
        &mut [], // no msos descriptors
        control_buf.as_mut_slice(),
    );

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: MouseReport::desc(),
        request_handler: Some(&mut request_handler),
        poll_ms: 60,
        max_packet_size: 8,
    };

    let mut writer = HidWriter::<_, 5>::new(&mut builder, &mut state, config);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let hid_fut = async {
        let mut y: i8 = 5;
        loop {
            Timer::after_millis(500).await;

            y = -y;
            let report = MouseReport {
                buttons: 0,
                x: 0,
                y,
                wheel: 0,
                pan: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => log::warn!("Failed to send report: {:?}", e),
            }
        }
    };

    log::info!("Hello, world!\nWe're going to wiggle your mouse along the X axis 🖱 ️↔️ ");
    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, hid_fut).await;
}
struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        log::info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        log::info!("Set report for {:?}: {:x?}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        log::info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        log::info!("Get idle rate for {:?}", id);
        None
    }
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Trace);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}
