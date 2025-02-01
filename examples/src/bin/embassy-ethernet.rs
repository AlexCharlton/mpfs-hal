#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;

#[macro_use]
extern crate mpfs_hal;

use mpfs_hal::ethernet::{EthernetDevice, MAC0};
use mpfs_hal::pac;
use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Starting");
    let mut device =
        EthernetDevice::new(MAC0::take().unwrap(), [0x02, 0x02, 0x02, 0x02, 0x02, 0x02]);
    run().await;
}

// Needs to be 8 byte aligned
static mut tx_pak_arp: [u8; 128] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC, 0x00, 0x12, 0x34, 0x56, 0x08, 0x06, 0x00, 0x01,
    0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xFC, 0x00, 0x12, 0x34, 0x56, 0x0A, 0x02, 0x02, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

async fn run() {
    unsafe {
        let mac = &mut pac::g_mac0;
        println!("Running");
        let mut test_speed = pac::__mss_mac_speed_t_MSS_MAC_1000MBPS;
        let mut test_fullduplex = 0;

        // Disable transmit while configuring queue
        (*mac.mac_base).NETWORK_CONTROL &= !pac::GEM_ENABLE_TRANSMIT;
        (*mac.mac_base).UPPER_TX_Q_BASE_ADDR = (&tx_pak_arp as *const _ as u64 >> 32) as u32;
        (*mac.mac_base).NETWORK_CONTROL |= pac::GEM_ENABLE_TRANSMIT;

        loop {
            let test_linkup =
                pac::MSS_MAC_get_link_status(mac, &mut test_speed, &mut test_fullduplex);
            println!(
                "Link status: {}; speed: {}; full duplex: {}",
                test_linkup, test_speed, test_fullduplex
            );

            if test_linkup != 0 {
                // Copy MAC address into the ARP packet
                tx_pak_arp[6..12].copy_from_slice(&mac.mac_addr);

                println!("Sending ARP",);

                let tx_status = pac::MSS_MAC_send_pkt(
                    mac,
                    0,
                    tx_pak_arp.as_ptr(),
                    tx_pak_arp.len() as u32,
                    core::ptr::null_mut(),
                );

                println!(
                    "send_pkt status: {}; Mac tx status: {:#?}",
                    tx_status,
                    (*mac.mac_base).TRANSMIT_STATUS
                );
            // println!("MAC queue 0: {:#?}", mac.queue[0]);
            } else {
                println!("Link not up");
            }
            Timer::after_millis(2000).await;
        }
    }
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Info);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
