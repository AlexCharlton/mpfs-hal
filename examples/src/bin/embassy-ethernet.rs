#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;

#[macro_use]
extern crate mpfs_hal;

use mpfs_hal::pac;

// Following along with https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples/blob/main/driver-examples/mss/mss-ethernet-mac/mpfs-mac-simple-test/src/application/hart0/e51.c#L2299
// Defines:
// - `MSS_MAC_HW_PLATFORM == MSS_MAC_DESIGN_BEAGLEV_FIRE_GEM0`
// - MSS_MAC_USE_PHY_RTL8211
// - TARGET_G5_SOC // Maybe? Seems needed to compile PAC

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Starting");

    // Print alignment info for our static buffers
    unsafe {
        let rx_ptr = &rx_buffer as *const _ as usize;
        let tx_ptr = &tx_pak_arp as *const _ as usize;

        println!(
            "rx_buffer offset for 4-byte align: {}",
            (4 - (rx_ptr & 0x3)) & 0x3
        );
        println!(
            "tx_pak_arp offset for 4-byte align: {}",
            (4 - (tx_ptr & 0x3)) & 0x3
        );
    }

    unsafe {
        (*pac::SYSREG).SOFT_RESET_CR = 0;
        (*pac::SYSREG).SUBBLK_CLOCK_CR = 0xFFFFFFFF;
    }

    low_level_init().await;

    /*
    display_clocks();
    prvLinkStatusTask();
    // ...
    // Respond to UART commands
    */
}

static mut rx_buffer: [[u8; pac::MSS_MAC_RX_RING_SIZE as usize];
    pac::MSS_MAC_MAX_RX_BUF_SIZE as usize] =
    [[0; pac::MSS_MAC_RX_RING_SIZE as usize]; pac::MSS_MAC_MAX_RX_BUF_SIZE as usize];

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

async fn low_level_init() {
    let mut config = pac::mss_mac_cfg_t {
        phy_addr: pac::PHY_RTL8211_MDIO_ADDR,
        phy_type: pac::MSS_MAC_DEV_PHY_RTL8211,
        pcs_phy_addr: pac::SGMII_MDIO_ADDR,
        interface_type: pac::TBI,
        phy_autonegotiate: Some(pac::MSS_MAC_RTL8211_phy_autonegotiate),
        phy_mac_autonegotiate: Some(pac::MSS_MAC_RTL8211_mac_autonegotiate),
        phy_get_link_status: Some(pac::MSS_MAC_RTL8211_phy_get_link_status),
        phy_init: Some(pac::MSS_MAC_RTL8211_phy_init),
        phy_set_link_speed: Some(pac::MSS_MAC_RTL8211_phy_set_link_speed),
        phy_extended_read: Some(pac::NULL_mmd_read_extended_regs),
        phy_extended_write: Some(pac::NULL_mmd_write_extended_regs),
        queue_enable: [0; 4],
        speed_mode: pac::__mss_mac_speed_mode_t_MSS_MAC_SPEED_AN,
        speed_duplex_select: pac::MSS_MAC_ANEG_ALL_SPEEDS,
        mac_addr: [0x00, 0xFC, 0x00, 0x12, 0x34, 0x56],
        phy_controller: core::ptr::null_mut(),
        tx_edc_enable: 0,
        rx_edc_enable: 0,
        jumbo_frame_enable: 0,
        jumbo_frame_default: pac::MSS_MAC_MAX_PACKET_SIZE,
        length_field_check: 1,
        append_CRC: 1,
        fullduplex: 0,
        loopback: 0,
        rx_flow_ctrl: 1,
        tx_flow_ctrl: 1,
        ipg_multiplier: pac::MSS_MAC_IPG_DEFVAL,
        ipg_divisor: pac::MSS_MAC_IPG_DEFVAL,
        phyclk: pac::MSS_MAC_DEF_PHY_CLK,
        max_frame_length: 0,
        use_hi_address: 0,
        use_local_ints: 0,
        queue0_int_priority: 7,
        queue1_int_priority: 7,
        queue2_int_priority: 7,
        queue3_int_priority: 7,
        mmsl_int_priority: 7,
        tsu_clock_select: 1,
        amba_burst_length: pac::MSS_MAC_AMBA_BURST_16,
    };

    unsafe {
        println!("Config initialized: {:#?}", config);
        #[allow(static_mut_refs)]
        let mac = &mut pac::g_mac0;
        pac::MSS_MAC_init(mac, &mut config as *mut _);

        (*mac.mac_base).NETWORK_CONTROL &= !0x8;
        // (*mac.mac_base).NETWORK_CONTROL &= !pac::GEM_ENABLE_TRANSMIT; // TODO
        /*
         * Not doing the tx disable/enable sequence here around the queue allocation
         * adjustment results in tx_go being set which causes the new tx code to
         * hang waiting for the last tx to complete...
         */

        /* Allocate all 4 segments to queue 0 as this is our only one... */
        (*mac.mac_base).TX_Q_SEG_ALLOC_Q0TO3 = 2;
        (*mac.mac_base).NETWORK_CONTROL |= 0x8;

        pac::MSS_MAC_set_tx_callback(mac, 0, Some(packet_tx_complete_handler));
        pac::MSS_MAC_set_rx_callback(mac, 0, Some(mac_rx_callback));

        // Allocate receive buffers for all slots in the ring
        for i in 0..pac::MSS_MAC_RX_RING_SIZE as usize {
            let enable_interrupts = if i == (pac::MSS_MAC_RX_RING_SIZE as usize - 1) {
                -1 // Enable interrupts for last buffer
            } else {
                0 // Keep interrupts disabled for all other buffers
            };

            pac::MSS_MAC_receive_pkt(
                mac,
                0,
                rx_buffer[i].as_mut_ptr(),
                core::ptr::null_mut(),
                enable_interrupts,
            );
        }

        pac::MSS_MAC_tx_enable(mac);
        println!("MAC initialized");

        // Copy MAC address into the ARP packet
        tx_pak_arp[6..12].copy_from_slice(&mac.mac_addr);

        let tx_status = pac::MSS_MAC_send_pkt(
            mac,
            0,
            tx_pak_arp.as_ptr(),
            tx_pak_arp.len() as u32,
            core::ptr::null_mut(),
        );

        println!("TX status: {}", tx_status);

        return;

        let mut last_rtl_regs = [0u16; 32];
        dump_rtl_regs(mac);
        println!("Initial RTL regs:");
        last_rtl_regs.copy_from_slice(&RTL_reg_0);
        RTL_reg_0
            .iter()
            .enumerate()
            .for_each(|(i, x)| println!("{:02}: {:#018b}", i, x));

        let mut test_speed = pac::__mss_mac_speed_t_MSS_MAC_1000MBPS;
        let mut test_fullduplex = 0;
        loop {
            let test_linkup =
                pac::MSS_MAC_get_link_status(mac, &mut test_speed, &mut test_fullduplex);
            println!(
                "Link status: {}; speed: {}; full duplex: {}",
                test_linkup, test_speed, test_fullduplex
            );
            // config
            //     .phy_mac_autonegotiate
            //     .map(|f| f(mac as *mut _ as *const core::ffi::c_void));
            // println!("Finished autonegotiation");
            dump_rtl_regs(mac);
            println!("Changed RTL regs:");
            RTL_reg_0
                .iter()
                .enumerate()
                .filter(|(i, &x)| x != last_rtl_regs[*i])
                .for_each(|(i, x)| {
                    println!("{:02}: {:#018b} (was {:#018b})", i, x, last_rtl_regs[i])
                });

            last_rtl_regs.copy_from_slice(&RTL_reg_0);

            // Copy MAC address into the ARP packet
            tx_pak_arp[6..12].copy_from_slice(&mac.mac_addr);

            let tx_status = pac::MSS_MAC_send_pkt(
                mac,
                0,
                tx_pak_arp.as_ptr(),
                tx_pak_arp.len() as u32,
                core::ptr::null_mut(),
            );

            println!("TX status: {}", tx_status);

            Timer::after_millis(5000).await;
        }
    }
    // TODO:
    // - Try sending something
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

extern "C" fn packet_tx_complete_handler(
    mac: *mut core::ffi::c_void,
    queue: u32,
    desc: *mut pac::mss_mac_tx_desc,
    marker: *mut core::ffi::c_void,
) {
    println!("Packet tx complete");
}

extern "C" fn mac_rx_callback(
    mac: *mut core::ffi::c_void,
    queue: u32,
    rx_buf: *mut u8,
    rx_size: u32,
    rx_desc: *mut pac::mss_mac_rx_desc,
    marker: *mut core::ffi::c_void,
) {
    println!("MAC rx callback");
    unsafe {
        pac::MSS_MAC_receive_pkt(mac as *mut _, 0, rx_buf, core::ptr::null_mut(), 1);
    }
}

// Declare the external C array and function
extern "C" {
    static mut RTL_reg_0: [u16; 32];
    fn dump_rtl_regs(this_mac: *const pac::mss_mac_instance_t);
}
