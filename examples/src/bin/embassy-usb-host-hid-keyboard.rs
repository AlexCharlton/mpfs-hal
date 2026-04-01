#![no_std]
#![no_main]

// https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_host_keyboard.rs used as reference

use embassy_usb_host::UsbHost;
use embassy_usb_host::class::kbd::KbdHandler;
use embassy_usb_host::handler::{EnumerationInfo, UsbHostHandler};
use mpfs_hal::Peripheral;
use mpfs_hal_embassy::usb::host;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");

    let driver = host::UsbHostDriver::take().unwrap();
    driver.start();
    let mut usbhost = UsbHost::new(driver);

    loop {
        log::debug!("Detecting device");
        // Wait for root-port to detect device
        let speed = usbhost.wait_for_connection().await;

        let mut config_buf = [0u8; 256];
        let result = usbhost.enumerate(speed, &mut config_buf).await;

        let (dev_desc, addr, _config_len) = match result {
            Ok(r) => r,
            Err(e) => {
                log::error!("Enumeration failed: {:?}", e);
                continue;
            }
        };
        let enum_info = EnumerationInfo {
            device_address: addr,
            ls_over_fs: false,
            speed: speed,
            device_desc: dev_desc,
        };

        log::info!(
            "Enumerated: VID={:04x} PID={:04x} addr={}",
            dev_desc.vendor_id,
            dev_desc.product_id,
            addr
        );
        let mut kbd = KbdHandler::try_register(usbhost.driver(), &enum_info)
            .await
            .expect("Couldn't register keyboard");

        loop {
            let result = kbd.wait_for_event().await;
            log::info!("Got interrupt: {:?}", result);
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
