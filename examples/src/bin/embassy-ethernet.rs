#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;

#[macro_use]
extern crate mpfs_hal;

use embassy_net_driver::{Driver, HardwareAddress, TxToken};
use mpfs_hal::ethernet::{EthernetDevice, MAC0};
use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Starting");
    let mut device =
        EthernetDevice::new(MAC0::take().unwrap(), [0x02, 0x02, 0x02, 0x02, 0x02, 0x02]);
    run(&mut device).await;
}

const TX_PAK_ARP: [u8; 128] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x00, 0x12, 0x34, 0x56, 0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xFC, 0x00, 0x12, 0x34, 0x56, 0x0A, 0x02, 0x02, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

use core::task::{RawWaker, RawWakerVTable, Waker};
// Define the vtable for our noop waker
const NOOP_WAKER_VTABLE: RawWakerVTable = RawWakerVTable::new(
    |_| RawWaker::new(core::ptr::null(), &NOOP_WAKER_VTABLE), // clone
    |_| {},                                                   // wake
    |_| {},                                                   // wake_by_ref
    |_| {},                                                   // drop
);

fn create_noop_waker() -> Waker {
    // Safety: The vtable contains valid function pointers and we never
    // use the data pointer for anything
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &NOOP_WAKER_VTABLE)) }
}

async fn run(device: &mut EthernetDevice<MAC0>) {
    println!("Running");
    let waker = create_noop_waker();
    let mut cx = core::task::Context::from_waker(&waker);
    let addr = device.hardware_address();
    let mac_addr = match addr {
        HardwareAddress::Ethernet(mac) => mac,
        _ => panic!("Expected MAC address"),
    };

    loop {
        let link_state = device.link_state(&mut cx);
        println!(
            "Link up? {:?}; Link speed: {:?}",
            link_state == embassy_net_driver::LinkState::Up,
            device.link_speed()
        );

        if link_state == embassy_net_driver::LinkState::Up {
            let tx_token = device.transmit(&mut cx).unwrap();
            tx_token.consume(TX_PAK_ARP.len(), |buf| {
                println!("Sending ARP",);
                buf.copy_from_slice(&TX_PAK_ARP);
                // Copy MAC address into the ARP packet
                buf[6..12].copy_from_slice(&mac_addr);
            });
        }
        Timer::after_millis(2000).await;
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
