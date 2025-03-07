#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};
use embassy_futures::join::join3;
use embassy_time::Timer;

#[macro_use]
extern crate mpfs_hal;
use mpfs_hal::ethernet::{EthernetDevice, MAC0};
use mpfs_hal::PeripheralRef;

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

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(spawner: embassy_executor::Spawner) {
    println!("Initializing ethernet");
    let device = EthernetDevice::<MAC0>::take().unwrap();
    device.init([0x02, 0x01, 0x02, 0x03, 0x04, 0x05]);
    spawner.spawn(run(device)).unwrap();
}

#[embassy_executor::task]
async fn run(device: &'static mut EthernetDevice<MAC0>) {
    let mac_addr = device.mac_address();
    let (mut tx, mut rx) = device.split();
    static LINK_UP: AtomicBool = AtomicBool::new(false);

    join3(
        async {
            loop {
                if LINK_UP.load(Ordering::Relaxed) {
                    rx.receive(|buffer| {
                        println!("Received packet of {} bytes", buffer.len());
                    })
                    .await;
                    log::trace!("Received packet");
                } else {
                    Timer::after_millis(10).await;
                }
            }
        },
        async {
            loop {
                if LINK_UP.load(Ordering::Relaxed) {
                    tx.send(TX_PAK_ARP.len(), |buf| {
                        println!("Sending ARP");
                        buf.copy_from_slice(&TX_PAK_ARP);
                        buf[6..12].copy_from_slice(&mac_addr);
                    });
                }
                Timer::after_millis(2000).await;
            }
        },
        async {
            loop {
                let previous_link_state = LINK_UP.load(Ordering::Relaxed);
                let link_state = device.link_state();
                if link_state.is_up() != previous_link_state {
                    LINK_UP.store(link_state.is_up(), Ordering::Relaxed);
                    println!(
                        "Link {:?}; Link speed: {:?}",
                        link_state,
                        device.link_speed()
                    );
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
    mpfs_hal::low_power_loop_forever()
}
