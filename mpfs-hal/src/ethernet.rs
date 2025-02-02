use core::task::Context;
use core::task::Waker;
use embassy_net_driver::{Capabilities, HardwareAddress, LinkState};
use paste::paste;

use crate::pac;
use crate::{Peripheral, PeripheralRef};

#[doc(hidden)]
#[repr(align(8))]
pub struct Buffer([u8; pac::MSS_MAC_MAX_RX_BUF_SIZE as usize]);

pub trait MacPeripheral: 'static + core::fmt::Debug + Peripheral {
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
        #[derive(Debug)]
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

        impl Peripheral for $MAC {
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

macro_rules! impl_eth {
    ($n:expr) => {
        paste! {
            impl_eth!([<ETH $n>], [<ETH $n _TAKEN>], [<MAC $n>]);
        }
    };

    // E.g. impl_mac!(MAC0, MAC0_TAKEN, MAC0_RX_BUFFER, MAC0_TX_BUFFER, g_mss_mac_0);
    ($ETH:ident, $ETH_TAKEN:ident, $MAC:ident) => {
        static mut $ETH: Option<EthernetDevice<$MAC>> = None;
        static mut $ETH_TAKEN: bool = false;

        impl PeripheralRef for EthernetDevice<$MAC> {
            fn take() -> Option<&'static mut Self> {
                critical_section::with(|_| unsafe {
                    if $ETH_TAKEN {
                        None
                    } else {
                        if let Some(mac) = $MAC::take() {
                            $ETH_TAKEN = true;
                            $ETH = Some(EthernetDevice::new(mac));
                            #[allow(static_mut_refs)]
                            Some($ETH.as_mut().unwrap())
                        } else {
                            None
                        }
                    }
                })
            }

            unsafe fn steal() -> &'static mut Self {
                $ETH = Some(EthernetDevice::new($MAC::steal()));
                #[allow(static_mut_refs)]
                $ETH.as_mut().unwrap()
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
    impl_eth!(0); // EthernetDevice<MAC0>

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
            jumbo_frame_enable: 1,
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

#[derive(Debug)]
pub struct EthernetDevice<M: MacPeripheral> {
    mac: M,
    rx_waker: Option<Waker>,
    // (index, length)
    pending_rx: RingBuffer<(usize, usize), { pac::MSS_MAC_RX_RING_SIZE as usize }>,
    rx_tokens_issued: usize,
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
    const fn new(mac: M) -> Self {
        Self {
            mac,
            pending_rx: RingBuffer {
                buffer: [(0, 0); pac::MSS_MAC_RX_RING_SIZE as usize],
                head: 0,
                tail: 0,
                size: 0,
            },
            rx_tokens_issued: 0,
            tx_buffer_in_use: false,
            rx_waker: None,
            link_speed: LinkSpeed::Speed10MBPS,
            full_duplex: false,
        }
    }

    pub fn init(&self, mac_address: [u8; 6]) {
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
    type RxToken<'a> = RxToken<'a, M>;
    type TxToken<'a> = TxToken<'a, M>;

    fn receive(&mut self, cx: &mut Context<'_>) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        critical_section::with(|_| {
            if self.rx_tokens_issued < self.pending_rx.size {
                self.rx_tokens_issued += 1;
                Some((
                    RxToken {
                        phantom: core::marker::PhantomData,
                        ethernet: self as *mut _,
                    },
                    self.transmit(cx).unwrap(),
                ))
            } else {
                self.rx_waker = Some(cx.waker().clone());
                None
            }
        })
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
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = 9216; // Max allowable according to Capabilities struct
        caps.max_burst_size = Some(pac::MSS_MAC_RX_RING_SIZE as usize);
        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        let mac = unsafe { *self.mac.address() };
        HardwareAddress::Ethernet(mac.mac_addr)
    }
}

pub struct RxToken<'a, M: MacPeripheral> {
    phantom: core::marker::PhantomData<&'a ()>,
    ethernet: *mut EthernetDevice<M>,
}

impl<'a, M: MacPeripheral> embassy_net_driver::RxToken for RxToken<'a, M> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let ethernet = unsafe { &mut *self.ethernet };

        let (buffer_number, len) = critical_section::with(|_| ethernet.pending_rx.pop().unwrap());
        let rx_buf = &mut ethernet.mac.rx_buffer(buffer_number).0[0..len];
        let ret = f(rx_buf);
        unsafe {
            pac::MSS_MAC_receive_pkt(
                ethernet.mac.address(),
                0,
                rx_buf.as_mut_ptr(),
                self.ethernet as *mut _,
                1,
            );
        }
        critical_section::with(|_| {
            ethernet.rx_tokens_issued -= 1;
        });
        ret
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
    log::trace!("MAC packet TX callback");
    unsafe {
        let ethernet = (user_data as *mut EthernetDevice<M>).as_mut().unwrap();
        ethernet.tx_buffer_in_use = false;
    }
}

extern "C" fn mac_rx_callback<M: MacPeripheral>(
    _mac: *mut core::ffi::c_void,
    _queue: u32,
    rx_buf: *mut u8,
    rx_size: u32,
    _rx_desc: *mut pac::mss_mac_rx_desc,
    user_data: *mut core::ffi::c_void,
) {
    log::trace!("MAC packet RX callback of size: {}", rx_size);
    unsafe {
        let ethernet = (user_data as *mut EthernetDevice<M>).as_mut().unwrap();
        let buffer_number = (rx_buf as usize - (*ethernet).mac.rx_buffer(0).0.as_ptr() as usize)
            / (pac::MSS_MAC_MAX_RX_BUF_SIZE as usize);
        let succeeded = critical_section::with(|_| {
            if let Some(waker) = (*ethernet).rx_waker.take() {
                waker.wake_by_ref();
            }
            (*ethernet)
                .pending_rx
                .push((buffer_number, rx_size as usize))
        });
        if !succeeded {
            log::warn!("Failed to push to pending rx");
            pac::MSS_MAC_receive_pkt(ethernet.mac.address(), 0, rx_buf, user_data as *mut _, 1);
        }
    }
}

//------------------------------------------------------
// Ring buffer
struct RingBuffer<T: core::fmt::Debug, const N: usize> {
    buffer: [T; N],
    head: usize,
    tail: usize,
    size: usize, // current number of elements
}

impl<T: core::fmt::Debug, const N: usize> core::fmt::Debug for RingBuffer<T, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "RingBuffer[")?;

        // let mut count = 0;
        // let mut idx = self.tail;
        // while count < self.size {
        //     if count > 0 {
        //         write!(f, ", ")?;
        //     }
        //     write!(f, "{:?}", self.buffer[idx])?;
        //     idx = (idx + 1) % N;
        //     count += 1;
        // }

        write!(
            f,
            "] (size: {}, head: {}, tail: {})",
            self.size, self.head, self.tail
        )
    }
}

impl<T: Default + Copy + core::fmt::Debug, const N: usize> RingBuffer<T, N> {
    #[allow(dead_code)]
    fn new() -> Self {
        Self {
            buffer: [T::default(); N],
            head: 0,
            tail: 0,
            size: 0,
        }
    }

    // Returns false if the buffer is full
    fn push(&mut self, item: T) -> bool {
        if self.is_full() {
            return false;
        }
        self.buffer[self.head] = item;
        self.head = (self.head + 1) % N;
        self.size += 1;
        true
    }

    fn is_full(&self) -> bool {
        self.size == N
    }

    fn is_empty(&self) -> bool {
        self.size == 0
    }

    fn pop(&mut self) -> Option<T> {
        if self.is_empty() {
            None
        } else {
            let item = self.buffer[self.tail];
            self.tail = (self.tail + 1) % N;
            self.size -= 1;
            Some(item)
        }
    }
}
