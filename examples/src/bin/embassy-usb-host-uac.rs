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
    println!("Enumerated device: {:?}", enum_info);

    if let Ok(uac) = UacHandler::try_register(&usbhost, enum_info).await {
        println!("UAC registered");
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
