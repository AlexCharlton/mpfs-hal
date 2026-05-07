#![no_std]
#![no_main]

// https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/usb_host_keyboard.rs used as reference

use embassy_usb_host::class::kbd::KbdHandler;
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

        let mut config_buf = [0u8; 256];
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

        log::info!(
            "Enumerated: VID={:04x} PID={:04x} addr={}",
            enum_info.device_desc.vendor_id,
            enum_info.device_desc.product_id,
            enum_info.device_address,
        );
        let mut kbd = KbdHandler::try_register(&bus, &enum_info)
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
