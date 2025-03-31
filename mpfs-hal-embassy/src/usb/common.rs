use embassy_usb_driver::{
    host::{ChannelError, HostError},
    Direction, EndpointType, Speed,
};
use mpfs_hal::pac;

// This should not be raised unless related constants in Endpoint Common are ammended
pub const NUM_ENDPOINTS: usize = 16;
pub const MAX_FIFO_SIZE: u16 = 4096;
pub const OUT_ENDPOINTS_START_ADDR: u16 = 0x0000;
pub const IN_ENDPOINTS_START_ADDR: u16 = 0x8000;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum EndpointState {
    Idle,
    Rx,
    Tx,
    TxLast,
    TxReadyForNext,
    Setup,
    RxComplete(Result<usize, ChannelError>),
    TxComplete,
    Disabled,
}

impl Default for EndpointState {
    fn default() -> Self {
        EndpointState::Disabled
    }
}

#[cfg(feature = "alloc")]
mod ep {
    extern crate alloc;

    use super::{default_ep, EndpointState, MAX_FIFO_SIZE};
    use aligned::{Aligned, A4};
    use alloc::boxed::Box;
    use core::task::Waker;
    use mpfs_hal::pac;

    #[derive(Debug)]
    pub struct EndpointController {
        pub ep: pac::mss_usb_ep_t,
        pub waker: Option<Waker>,
        pub state: EndpointState,
        pub buffer: Option<Box<Aligned<A4, [u8; MAX_FIFO_SIZE as usize]>>>,
    }

    impl EndpointController {
        pub fn default() -> Self {
            EndpointController {
                ep: default_ep(),
                waker: None,
                state: EndpointState::Disabled,
                buffer: None,
            }
        }
        pub fn buffer_addr(&mut self) -> &mut [u8] {
            if self.buffer.is_none() {
                self.buffer = Some(Box::new(Aligned::<A4, _>([0; MAX_FIFO_SIZE as usize])));
            }
            self.buffer.as_mut().unwrap().as_mut_slice()
        }
    }
}

#[cfg(not(feature = "alloc"))]
mod ep {
    use super::{default_ep, EndpointState};
    use core::task::Waker;
    use mpfs_hal::pac;

    #[derive(Debug)]
    pub struct EndpointController {
        pub ep: pac::mss_usb_ep_t,
        pub waker: Option<Waker>,
        pub state: EndpointState,
    }

    impl EndpointController {
        pub fn default() -> Self {
            EndpointController {
                ep: default_ep(),
                waker: None,
                state: EndpointState::Disabled,
            }
        }
        pub fn buffer_addr(&mut self) -> &mut [u8] {
            panic!("USB buffers that aren't 4-byte aligned are not supported without alloc");
        }
    }
}

pub use ep::*;

// Endpoints need to have a FIFO assigned to an address in the MSS USB internal RAM.
// The RAM is 65k (0xFFFF) bytes, so using 32 endpoints (16 in + 16 out)
// allows us to give 2048 bytes to each endpoint if we devide it equally. We will allow
// the user to configure the endpoint with a max packet size of up to 4096 bytes, however
// which means that not all endpoints will be able to have the maximum packet size.
// Further, we will devide the FIFO into 2 parts, with 0x0000 being the start address for
// the out endpoints and 0x8000 being the start address for the in endpoints.
//
// Additionally, we have 4 DMA channels available, and they can each only be mapped to
// one endpoint. Because of this we will configure channels 1 and 2 (for both in and out)
// to use DMA. DMA channel 1, 2 will be used for Out endpoints and DMA channel 3, 4 will
// be used for In endpoints.
//
// Reference: mss_usb_device.h

#[derive(Default, Copy, Clone)]
pub struct EndpointDetails {
    pub used: bool,
    pub fifo_addr: u16,
    pub fifo_size: u16,
}

// We allocate in and out endpoints using the same address space, and then offset in the IN
// endpoints in `configure_endpoint_controller`.
pub fn alloc_fifo_addr(
    max_packet_size: u16,
    endpoints: &mut [EndpointDetails],
) -> Result<usize, HostError> {
    if max_packet_size > MAX_FIFO_SIZE {
        return Err(HostError::OutOfChannels);
    }
    // FIFO size must be a power of 2
    let mut fifo_size = max_packet_size.next_power_of_two();
    if fifo_size < 1024 {
        // If we're using a small enough packet size, we increase the FIFO size
        // to allow for double packet buffering
        fifo_size = (fifo_size + 1).next_power_of_two();
    }
    if let Some(i) = endpoints.iter().position(|x| !x.used) {
        let fifo_addr = if i == 0 {
            0
        } else {
            endpoints[i - 1].fifo_addr + endpoints[i - 1].fifo_size
        };
        if fifo_addr + fifo_size > IN_ENDPOINTS_START_ADDR {
            log::error!(
                "FIFO address is out of bounds for out endpoint {:?} with size {:?}",
                i,
                max_packet_size
            );
            return Err(HostError::OutOfChannels);
        }
        endpoints[i] = EndpointDetails {
            used: true,
            fifo_addr,
            fifo_size,
        };
        Ok(i)
    } else {
        Err(HostError::OutOfChannels)
    }
}

pub fn configure_endpoint_controller(
    direction: Direction,
    endpoint: usize,
    endpoint_type: EndpointType,
    max_packet_size: u16,
    endpoint_details: EndpointDetails,
    speed: Speed,
) -> EndpointController {
    // Get the standard max packet size based on speed and transfer type
    let std_max_pkt_sz = if speed == Speed::High {
        match endpoint_type {
            EndpointType::Bulk => pac::USB_HS_BULK_MAX_PKT_SIZE,
            EndpointType::Interrupt => pac::USB_HS_INTERRUPT_MAX_PKT_SIZE,
            EndpointType::Isochronous => pac::USB_HS_ISO_MAX_PKT_SIZE,
            EndpointType::Control => pac::USB_HS_BULK_MAX_PKT_SIZE, // Control uses same as bulk
        }
    } else if speed == Speed::Full {
        match endpoint_type {
            EndpointType::Bulk => pac::USB_FS_BULK_MAX_PKT_SIZE,
            EndpointType::Interrupt => pac::USB_FS_INTERRUPT_MAX_PKT_SIZE,
            EndpointType::Isochronous => pac::USB_FS_ISO_MAX_PKT_SIZE,
            EndpointType::Control => pac::USB_FS_BULK_MAX_PKT_SIZE,
        }
    } else {
        panic!("Unsupported speed: {:?}", speed);
    };

    if max_packet_size > std_max_pkt_sz as u16 {
        panic!("Max packet size is greater than the allowed max packet size for this transfer type {} expected for {:?}, got {}", std_max_pkt_sz, endpoint_type, max_packet_size);
    }

    let fifo_addr = endpoint_details.fifo_addr
        + if direction == Direction::Out {
            OUT_ENDPOINTS_START_ADDR
        } else {
            IN_ENDPOINTS_START_ADDR
        };

    let dma_channel = endpoint_to_dma_channel(endpoint, direction);
    let dma_enable = dma_channel != pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL_NA;

    // Configure double packet buffering if there's enough FIFO space reserved
    let dpb_enable = max_packet_size * 2 <= endpoint_details.fifo_size;

    let ep = pac::mss_usb_ep_t {
        num: endpoint as u32,
        xfr_type: mss_transfer_type(endpoint_type),
        max_pkt_size: max_packet_size,
        fifo_size: endpoint_details.fifo_size,
        fifo_addr,
        dma_enable: dma_enable as u8,
        dpb_enable: dpb_enable as u8,
        dma_channel,
        ..default_ep()
    };

    EndpointController {
        ep,
        ..EndpointController::default()
    }
}

pub fn mss_transfer_type(endpoint_type: EndpointType) -> pac::mss_usb_xfr_type_t {
    match endpoint_type {
        EndpointType::Bulk => pac::mss_usb_xfr_type_t_MSS_USB_XFR_BULK,
        EndpointType::Interrupt => pac::mss_usb_xfr_type_t_MSS_USB_XFR_INTERRUPT,
        EndpointType::Isochronous => pac::mss_usb_xfr_type_t_MSS_USB_XFR_ISO,
        EndpointType::Control => pac::mss_usb_xfr_type_t_MSS_USB_XFR_CONTROL,
    }
}

fn endpoint_to_dma_channel(endpoint: usize, direction: Direction) -> pac::mss_usb_dma_channel_t {
    match (endpoint, direction) {
        (1, Direction::In) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL1,
        (2, Direction::In) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL2,
        (1, Direction::Out) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL3,
        (2, Direction::Out) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL4,
        _ => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL_NA,
    }
}

pub fn speed_to_mss_value(speed: Speed) -> pac::mss_usb_device_speed_t {
    match speed {
        Speed::Full => pac::mss_usb_device_speed_t_MSS_USB_DEVICE_FS,
        Speed::High => pac::mss_usb_device_speed_t_MSS_USB_DEVICE_HS,
        Speed::Low => pac::mss_usb_device_speed_t_MSS_USB_DEVICE_LS,
    }
}

pub const fn default_ep() -> pac::mss_usb_ep_t {
    pac::mss_usb_ep_t {
        // Important for configuration
        num: 0,
        xfr_type: pac::mss_usb_xfr_type_t_MSS_USB_XFR_CONTROL,
        max_pkt_size: 0,
        fifo_size: 0,
        fifo_addr: 0,
        dma_enable: 0,
        dpb_enable: 0,
        dma_channel: pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL_NA,

        // Used for transfer
        buf_addr: core::ptr::null_mut(),
        txn_length: 0,
        xfr_length: 0,
        txn_count: 0,
        xfr_count: 0,
        state: pac::mss_usb_ep_state_t_MSS_USB_EP_VALID,

        // Used for configuration by the high-level driver, but not by this
        add_zlp: 0,     // Used by the high-level driver that we don't use
        num_usb_pkt: 1, // This must always be 1, according to mss_usb_device.h
        cep_data_dir: 0,

        // Unused?
        cep_cmd_addr: core::ptr::null_mut(),
        disable_ping: 0,
        interval: 0,
        stall: 0,
        req_pkt_n: 0,
        tdev_idx: 0,
    }
}
