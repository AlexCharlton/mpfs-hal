use core::task::Context;
use core::task::Waker;
use embassy_net_driver::{Capabilities, HardwareAddress, LinkState};
use paste::paste;

use crate::pac;

#[doc(hidden)]
#[repr(align(8))]
pub struct Buffer([u8; pac::MSS_MAC_MAX_RX_BUF_SIZE as usize]);

pub trait MacPeripheral: 'static {
    #[doc(hidden)]
    fn address(&self) -> *mut pac::mss_mac_instance;
    #[doc(hidden)]
    fn tx_buffer(&self) -> &'static mut Buffer;
    #[doc(hidden)]
    fn rx_buffer(&self, index: usize) -> &'static mut Buffer;
}

//-------------------------------------------------------------------
// Create the MAC peripherals
// Maybe TODO: We're reserving a moderate amount of memory for the RX and TX buffers
// (10kB x 17 buffers = 170kB x num MACs)
// This should probably be opt-in.

macro_rules! impl_mac {
    ($n:expr) => {
        paste! {
            impl_mac!([<MAC $n>], [<MAC $n _TAKEN>], [<MAC_RX_BUFFER $n>], [<MAC_TX_BUFFER $n>], [<g_mac $n>]);
        }
    };

    // E.g. impl_mac!(MAC0, MAC0_TAKEN, MAC0_RX_BUFFER, MAC0_TX_BUFFER, g_mss_mac_0);
    ($MAC:ident, $MAC_TAKEN:ident, $RX_BUFFER:ident, $TX_BUFFER:ident, $instance:ident) => {
        pub struct $MAC {
            _private: (),
        }
        static mut $MAC_TAKEN: bool = false;
        static mut $RX_BUFFER: [Buffer; pac::MSS_MAC_RX_RING_SIZE as usize] =
    [const { Buffer([0; pac::MSS_MAC_MAX_RX_BUF_SIZE as usize]) };
        pac::MSS_MAC_RX_RING_SIZE as usize];

        // We're only going to support a single TX at a time, since the platform doesn't
        // _really_ support multiple TXs.
        static mut $TX_BUFFER: Buffer = Buffer([0; pac::MSS_MAC_MAX_TX_BUF_SIZE as usize]);

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

            fn tx_buffer(&self) -> &'static mut Buffer {
                #[allow(static_mut_refs)]
                unsafe { &mut $TX_BUFFER }
            }

            fn rx_buffer(&self, index: usize) -> &'static mut Buffer {
                unsafe { &mut $RX_BUFFER[index] }
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

#[derive(Debug, Clone, Copy)]
pub enum LinkSpeed {
    Speed10MBPS,
    Speed100MBPS,
    Speed1000MBPS,
}

pub struct EthernetDevice<M: MacPeripheral> {
    pub mac: M,
    rx_waker: Option<Waker>,
    rx_read_index: usize,
    rx_write_index: usize,
    rx_lengths: [usize; pac::MSS_MAC_RX_RING_SIZE as usize],
    tx_buffer_in_use: bool,
    link_speed: LinkSpeed,
    full_duplex: bool,
}

impl<M: MacPeripheral> EthernetDevice<M> {
    pub fn link_speed(&self) -> LinkSpeed {
        self.link_speed
    }

    pub fn full_duplex(&self) -> bool {
        self.full_duplex
    }

    #[doc(hidden)]
    pub fn debug_mac_queue(&self) {
        // println!("MAC queue 0: {:#?}", mac.queue[0]);
    }
}

impl<M: MacPeripheral> EthernetDevice<M> {
    pub fn new(mac: M, mac_address: [u8; 6]) -> Self {
        let device = Self {
            mac,
            rx_read_index: 0,
            rx_lengths: [0; pac::MSS_MAC_RX_RING_SIZE as usize],
            rx_write_index: 0,
            tx_buffer_in_use: false,
            rx_waker: None,
            link_speed: LinkSpeed::Speed10MBPS,
            full_duplex: false,
        };
        device.configure(mac_address);
        device
    }

    fn configure(&self, mac_address: [u8; 6]) {
        unsafe {
            let mut config = default_mac_config();
            config.mac_addr = mac_address;
            pac::MSS_MAC_init(self.mac.address(), &mut config as *mut _);

            pac::MSS_MAC_set_tx_callback(
                self.mac.address(),
                0,
                Some(packet_tx_complete_handler::<M>),
            );
            pac::MSS_MAC_set_rx_callback(self.mac.address(), 0, Some(mac_rx_callback::<M>));

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
                    self.mac.rx_buffer(i).0.as_mut_ptr(),
                    self as *const _ as *mut core::ffi::c_void,
                    enable_interrupts,
                );
            }

            {
                // Set up the most significant 32 bits of the tx buffer address
                // This is done in platform for the RX, but it's omitted for the TX, for some reason.
                let mac_base = (*self.mac.address()).mac_base;
                // Disable transmit while configuring queue
                (*mac_base).NETWORK_CONTROL &= !pac::GEM_ENABLE_TRANSMIT;
                (*mac_base).UPPER_TX_Q_BASE_ADDR =
                    (self.mac.tx_buffer().0.as_ptr() as u64 >> 32) as u32;
                (*mac_base).NETWORK_CONTROL |= pac::GEM_ENABLE_TRANSMIT;
            }

            pac::MSS_MAC_tx_enable(self.mac.address());

            log::trace!("MAC config: {:#?}", config);
            log::debug!("MAC initialized");
        }
    }

    // The packet is expected to be in the tx buffer already
    fn send_pkt(&mut self, len: usize) {
        unsafe {
            let tx_status = pac::MSS_MAC_send_pkt(
                self.mac.address(),
                0,
                self.mac.tx_buffer().0.as_ptr(),
                len as u32,
                self as *const _ as *mut core::ffi::c_void,
            );
            if tx_status != 1 {
                log::error!("Failed to send packet");
            }
        }
    }
}

impl<M: MacPeripheral> embassy_net_driver::Driver for EthernetDevice<M> {
    type RxToken<'a> = RxToken<'a>;
    type TxToken<'a> = TxToken<'a, M>;

    fn receive(&mut self, cx: &mut Context<'_>) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        todo!()
    }

    fn transmit(&mut self, _cx: &mut Context<'_>) -> Option<Self::TxToken<'_>> {
        // We can actually create as many TxTokens as we want: What matters is that we don't consume
        // more than one at a time. Consume will take the lock on ethernet.
        Some(TxToken {
            phantom: core::marker::PhantomData,
            ethernet: self as *mut _,
        })
    }

    fn link_state(&mut self, cx: &mut Context<'_>) -> LinkState {
        cx.waker().wake_by_ref();
        let mut test_speed = pac::__mss_mac_speed_t_MSS_MAC_1000MBPS;
        let mut test_fullduplex = 0;
        let test_linkup = unsafe {
            pac::MSS_MAC_get_link_status(self.mac.address(), &mut test_speed, &mut test_fullduplex)
        };

        if test_linkup == 0 {
            return LinkState::Down;
        }
        if test_speed == pac::__mss_mac_speed_t_MSS_MAC_1000MBPS {
            self.link_speed = LinkSpeed::Speed1000MBPS;
        } else if test_speed == pac::__mss_mac_speed_t_MSS_MAC_100MBPS {
            self.link_speed = LinkSpeed::Speed100MBPS;
        } else {
            self.link_speed = LinkSpeed::Speed10MBPS;
        }

        self.full_duplex = test_fullduplex != 0;

        LinkState::Up
    }

    fn capabilities(&self) -> Capabilities {
        todo!()
    }

    fn hardware_address(&self) -> HardwareAddress {
        let mac = unsafe { *self.mac.address() };
        HardwareAddress::Ethernet(mac.mac_addr)
    }
}

pub struct RxToken<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
    length: usize,
}

impl<'a> embassy_net_driver::RxToken for RxToken<'a> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        todo!()
    }
}

pub struct TxToken<'a, M: MacPeripheral> {
    phantom: core::marker::PhantomData<&'a ()>,
    ethernet: *mut EthernetDevice<M>,
}

impl<'a, M: MacPeripheral> embassy_net_driver::TxToken for TxToken<'a, M> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let ethernet = unsafe { &mut *self.ethernet };
        let mut lock_count = 0;
        loop {
            if !ethernet.tx_buffer_in_use {
                let got_lock = critical_section::with(|_| {
                    // Check again, since we might have lost the lock
                    if !ethernet.tx_buffer_in_use {
                        ethernet.tx_buffer_in_use = true;
                        true
                    } else {
                        false
                    }
                });
                if got_lock {
                    break;
                }
            }
            lock_count += 1;

            // Clear the lock if we get stuck here, since it's possible a bad packet will never get its callback called.
            if lock_count > 1000 {
                ethernet.tx_buffer_in_use = false;
                log::warn!("Failed to get lock on ethernet TX buffer, resetting");
            }
        }
        if len > ethernet.mac.tx_buffer().0.len() {
            panic!(
                "Max ethernet TX packet size {} exceeded",
                ethernet.mac.tx_buffer().0.len()
            );
        }
        // We own the tx buffer now, so we can send the packet
        let ret = f(&mut ethernet.mac.tx_buffer().0[0..len]);
        ethernet.send_pkt(len);

        ret
    }
}

extern "C" fn packet_tx_complete_handler<M: MacPeripheral>(
    _mac: *mut core::ffi::c_void,
    _queue: u32,
    _desc: *mut pac::mss_mac_tx_desc,
    user_data: *mut core::ffi::c_void,
) {
    unsafe {
        let ethernet = (user_data as *mut EthernetDevice<M>).as_mut().unwrap();
        ethernet.tx_buffer_in_use = false;
    }
    log::trace!("Packet tx complete");
}

extern "C" fn mac_rx_callback<M: MacPeripheral>(
    mac: *mut core::ffi::c_void,
    _queue: u32,
    rx_buf: *mut u8,
    rx_size: u32,
    _rx_desc: *mut pac::mss_mac_rx_desc,
    user_data: *mut core::ffi::c_void,
) {
    log::info!("MAC rx packet of size: {}", rx_size);
    // unsafe {
    //     let data: &[u8] = core::slice::from_raw_parts(rx_buf, rx_size as usize);
    //     if rx_size >= 32 {
    //         log::info!("  first 32 bytes: {:?}", &data[..32]);
    //     }
    // }
    unsafe {
        let ethernet = user_data as *mut EthernetDevice<M>;
        // This shouldn't be called until we've consumed the packet
        pac::MSS_MAC_receive_pkt(mac as *mut _, 0, rx_buf, user_data, 1);
    }
}
