use crate::pac;
use paste::paste;

pub trait MacPeripheral {
    #[doc(hidden)]
    fn address(&self) -> *mut pac::mss_mac_instance;
}

//-------------------------------------------------------------------
// Create the MAC peripherals

macro_rules! impl_mac {
    ($n:expr) => {
        paste! {
            impl_mac!([<MAC $n>], [<MAC $n _TAKEN>], $n, [<g_mac $n>]);
        }
    };

    // E.g. impl_mac!(MAC0, MAC0_TAKEN, 0, g_mss_mac_0);
    ($MAC:ident, $MAC_TAKEN:ident, $num:expr, $instance:ident) => {
        pub struct $MAC {
            _private: (),
        }
        static mut $MAC_TAKEN: bool = false;

        impl crate::Peripheral for $MAC {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $MAC_TAKEN {
                        None
                    } else {
                        $MAC_TAKEN = true;
                        Some(Self { _private: () })
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }

        impl MacPeripheral for $MAC {
            fn address(&self) -> *mut pac::mss_mac_instance {
                core::ptr::addr_of_mut!(pac::$instance)
            }
        }
    };
}

#[cfg(feature = "beaglev-fire")]
pub use beaglev_fire::*;
#[cfg(feature = "beaglev-fire")]
mod beaglev_fire {
    use super::*;

    impl_mac!(0); // MAC0

    pub(crate) fn default_mac_config() -> pac::mss_mac_cfg_t {
        pac::mss_mac_cfg_t {
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
            mac_addr: [0; 6],
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
        }
    }
}

//-------------------------------------------------------------------
// Ethernet

// Needs to be 8 byte aligned
static mut RX_BUFFER: [[u8; pac::MSS_MAC_RX_RING_SIZE as usize];
    pac::MSS_MAC_MAX_RX_BUF_SIZE as usize] =
    [[0; pac::MSS_MAC_RX_RING_SIZE as usize]; pac::MSS_MAC_MAX_RX_BUF_SIZE as usize];

pub struct EthernetDevice<M: MacPeripheral> {
    mac: M,
}

impl<M: MacPeripheral> EthernetDevice<M> {
    pub fn new(mac: M, mac_address: [u8; 6]) -> Self {
        let device = Self { mac };
        device.configure(mac_address);
        device
    }

    pub fn link_status(&self) -> bool {
        todo!()
    }

    fn configure(&self, mac_address: [u8; 6]) {
        unsafe {
            let mut config = default_mac_config();
            config.mac_addr = mac_address;
            pac::MSS_MAC_init(self.mac.address(), &mut config as *mut _);

            // // Disable transmit while configuring queue
            // (*mac.mac_base).NETWORK_CONTROL &= !pac::GEM_ENABLE_TRANSMIT;
            // (*mac.mac_base).UPPER_TX_Q_BASE_ADDR = (&tx_pak_arp as *const _ as u64 >> 32) as u32;
            // (*mac.mac_base).NETWORK_CONTROL |= pac::GEM_ENABLE_TRANSMIT;

            pac::MSS_MAC_set_tx_callback(self.mac.address(), 0, Some(packet_tx_complete_handler));
            pac::MSS_MAC_set_rx_callback(self.mac.address(), 0, Some(mac_rx_callback));

            // Allocate receive buffers for all slots in the ring
            for i in 0..pac::MSS_MAC_RX_RING_SIZE as usize {
                let enable_interrupts = if i == (pac::MSS_MAC_RX_RING_SIZE as usize - 1) {
                    -1 // Enable interrupts for last buffer
                } else {
                    0 // Keep interrupts disabled for all other buffers
                };

                pac::MSS_MAC_receive_pkt(
                    self.mac.address(),
                    0,
                    RX_BUFFER[i].as_mut_ptr(),
                    core::ptr::null_mut(),
                    enable_interrupts,
                );
            }

            pac::MSS_MAC_tx_enable(self.mac.address());

            log::trace!("MAC config: {:#?}", config);
            log::debug!("MAC initialized");
        }
    }
}

extern "C" fn packet_tx_complete_handler(
    _mac: *mut core::ffi::c_void,
    _queue: u32,
    _desc: *mut pac::mss_mac_tx_desc,
    _marker: *mut core::ffi::c_void,
) {
    log::info!("Packet tx complete");
}

extern "C" fn mac_rx_callback(
    mac: *mut core::ffi::c_void,
    _queue: u32,
    rx_buf: *mut u8,
    rx_size: u32,
    _rx_desc: *mut pac::mss_mac_rx_desc,
    _marker: *mut core::ffi::c_void,
) {
    log::info!("MAC rx packet of size: {}", rx_size);
    unsafe {
        pac::MSS_MAC_receive_pkt(mac as *mut _, 0, rx_buf, core::ptr::null_mut(), 1);
    }
}
