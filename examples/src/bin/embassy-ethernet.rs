#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;

#[macro_use]
extern crate mpfs_hal;

use embassy_futures::join::join3;
use embassy_net_driver::{Driver, HardwareAddress, RxToken, TxToken};
use mpfs_hal::ethernet::{EthernetDevice, MAC0};
use mpfs_hal::PeripheralRef;

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(spawner: embassy_executor::Spawner) {
    println!("Starting");
    let device = EthernetDevice::<MAC0>::take().unwrap();
    device.init([0x02, 0x02, 0x02, 0x02, 0x02, 0x02]);
    spawner.spawn(run(device)).unwrap();
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

#[embassy_executor::task]
async fn run(device: &'static mut EthernetDevice<MAC0>) {
    let waker = create_noop_waker();
    let mac_addr = match device.hardware_address() {
        HardwareAddress::Ethernet(mac) => mac,
        _ => panic!("Expected MAC address"),
    };
    let device: Mutex<CriticalSectionRawMutex, _> = Mutex::new(device);
    static LINK_UP: AtomicBool = AtomicBool::new(false);

    join3(
        async {
            let mut cx = core::task::Context::from_waker(&waker);
            loop {
                if LINK_UP.load(Ordering::Relaxed) {
                    let mut device = device.lock().await;
                    if let Some((rx_token, _)) = device.receive(&mut cx) {
                        rx_token.consume(|buffer| {
                            println!("Received packet of {} bytes", buffer.len());
                        });
                    }
                }
                Timer::after_millis(10).await;
            }
        },
        async {
            let mut cx = core::task::Context::from_waker(&waker);
            loop {
                if LINK_UP.load(Ordering::Relaxed) {
                    let mut device = device.lock().await;
                    if let Some(tx_token) = device.transmit(&mut cx) {
                        tx_token.consume(TX_PAK_ARP.len(), |buf| {
                            println!("Sending ARP");
                            buf.copy_from_slice(&TX_PAK_ARP);
                            buf[6..12].copy_from_slice(&mac_addr);
                        });
                    }
                }
                Timer::after_millis(2000).await;
            }
        },
        async {
            let mut cx = core::task::Context::from_waker(&waker);
            loop {
                {
                    let mut device = device.lock().await;
                    let link_state = device.link_state(&mut cx);
                    let is_up = link_state == embassy_net_driver::LinkState::Up;
                    let previous_link_state = LINK_UP.load(Ordering::Relaxed);
                    if is_up != previous_link_state {
                        LINK_UP.store(is_up, Ordering::Relaxed);
                        println!(
                            "Link up? {:?}; Link speed: {:?}",
                            is_up,
                            device.link_speed()
                        );
                    }
                }
                Timer::after_millis(250).await;
            }
        },
    )
    .await;
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Debug);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}

//--------------------------------------------------------------
// Noop waker; Use Waker::noop() in 1.85
use core::task::{RawWaker, RawWakerVTable, Waker};
const NOOP_WAKER_VTABLE: RawWakerVTable = RawWakerVTable::new(
    |_| RawWaker::new(core::ptr::null(), &NOOP_WAKER_VTABLE),
    |_| {},
    |_| {},
    |_| {},
);

fn create_noop_waker() -> Waker {
    unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &NOOP_WAKER_VTABLE)) }
}
