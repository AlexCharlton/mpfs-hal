#![no_std]
#![no_main]

extern crate alloc;

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
    println!("Hello");
    /*
    void mac_task(void *pvParameters) {
        SYSREG->SOFT_RESET_CR = 0U;
        SYSREG->SUBBLK_CLOCK_CR = 0xFFFFFFFFUL;
    */

    low_level_init();

    /*
    display_clocks();
    prvLinkStatusTask();
    // ...
    // Respond to UART commands
    */
}

fn low_level_init() {
    let mut config = pac::mss_mac_cfg_t {
        interface_type: pac::TBI,
        phy_type: pac::MSS_MAC_DEV_PHY_RTL8211,
        phy_init: Some(pac::MSS_MAC_RTL8211_phy_init),
        phy_set_link_speed: Some(pac::MSS_MAC_RTL8211_phy_set_link_speed),
        phy_autonegotiate: Some(pac::MSS_MAC_RTL8211_phy_autonegotiate),
        phy_mac_autonegotiate: Some(pac::MSS_MAC_RTL8211_mac_autonegotiate),
        phy_get_link_status: Some(pac::MSS_MAC_RTL8211_phy_get_link_status),
        phy_extended_read: Some(pac::NULL_mmd_read_extended_regs),
        phy_extended_write: Some(pac::NULL_mmd_write_extended_regs),
        queue_enable: [0; 4],
        speed_mode: pac::__mss_mac_speed_mode_t_MSS_MAC_SPEED_AN,
        speed_duplex_select: pac::MSS_MAC_ANEG_ALL_SPEEDS,
        mac_addr: [0x00, 0xFC, 0x00, 0x12, 0x34, 0x56],
        phy_addr: pac::PHY_RTL8211_MDIO_ADDR,
        pcs_phy_addr: pac::SGMII_MDIO_ADDR,
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
        pac::MSS_MAC_cfg_struct_def_init(&mut config);
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

        //pac::MSS_MAC_tx_enable(mac);
    }

    println!("MAC initialized");
    /*

        /*
         * Allocate receive buffers.
         *
         * We prime the pump with a full set of packet buffers and then re use them
         * as each packet is handled.
         *
         * This function will need to be called each time a packet is received to
         * hand back the receive buffer to the MAC driver.
         */
        for (count = 0; count < MSS_MAC_RX_RING_SIZE; ++count)
        {
            /*
             * We allocate the buffers with the Ethernet MAC interrupt disabled
             * until we get to the last one. For the last one we ensure the Ethernet
             * MAC interrupt is enabled on return from MSS_MAC_receive_pkt().
             */
    #if defined(MSS_MAC_USE_DDR)
            if (count != (MSS_MAC_RX_RING_SIZE - 1))
            {
                MSS_MAC_receive_pkt(g_test_mac,
                                    0,
                                    g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                    0,
                                    0);
            }
            else
            {
                MSS_MAC_receive_pkt(g_test_mac,
                                    0,
                                    g_mac_rx_buffer + count * MSS_MAC_MAX_RX_BUF_SIZE,
                                    0,
                                    -1);
            }
    #else
            if (count != (MSS_MAC_RX_RING_SIZE - 1))
            {
                MSS_MAC_receive_pkt(g_test_mac, 0, g_mac_rx_buffer[count], 0, 0);
            }
            else
            {
                MSS_MAC_receive_pkt(g_test_mac, 0, g_mac_rx_buffer[count], 0, -1);
            }
    #endif
        }
         */
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
    rx_buffer: *mut u8,
    rx_size: u32,
    rx_desc: *mut pac::mss_mac_rx_desc,
    marker: *mut core::ffi::c_void,
) {
    println!("MAC rx callback");
}
