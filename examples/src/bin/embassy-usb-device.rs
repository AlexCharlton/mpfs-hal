#![no_std]
#![no_main]

use mpfs_hal::Peripheral;

#[macro_use]
extern crate mpfs_hal;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let _usb_driver = mpfs_hal::usb::device::UsbDriver::take().unwrap();
    println!("Hello, world!\nWe're going to wiggle your mouse along the X axis 🖱 ️↔️ ");
    loop {
        // unsafe {
        //     if pac::MSS_USBD_HID_tx_done() == 1 {
        //         #[allow(static_mut_refs)]
        //         pac::MSS_USBD_HID_tx_report(
        //             &REPORT as *const _ as *mut u8,
        //             core::mem::size_of::<pac::mss_usbd_hid_report_t>() as u32,
        //         );
        //     } else {
        //         // Delay
        //         for _ in 0..50000 {
        //             core::hint::spin_loop();
        //         }

        //         if REPORT.x_move < 30 {
        //             REPORT.x_move += 1;
        //         } else {
        //             REPORT.x_move = -30;
        //         }
        //     }
        // }
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
