#![no_std]
#![no_main]

// https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_midi.rs used as reference

use embassy_usb::handlers::{kbd::KbdHandler, UsbHostHandler};
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::host::{DeviceEvent::Connected, UsbHostDriver};
use mpfs_hal::usb::host::UsbHost;
use mpfs_hal::Peripheral;

#[macro_use]
extern crate mpfs_hal;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");

    let mut usbhost = UsbHost::take().unwrap();

    usbhost.start();

    log::debug!("Detecting device");
    // Wait for root-port to detect device
    let speed = loop {
        match usbhost.wait_for_device_event().await {
            Connected(speed) => break speed,
            _ => {}
        }
    };

    println!("Found device with speed = {:?}", speed);

    let enum_info = usbhost.enumerate_root(speed, 1).await.unwrap();
    let mut kbd = KbdHandler::try_register(&usbhost, enum_info)
        .await
        .expect("Couldn't register keyboard");

    loop {
        let result = kbd.wait_for_event().await;
        log::debug!("{:?}", result);
    }
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Trace);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
