use crate::Mutex;

use aligned::{Aligned, A4};
use core::future::poll_fn;
use core::task::{Poll, Waker};
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType,
    Event,
};

use crate::{pac, Peripheral};

// This should not be raised unless related constants in Endpoint Common are ammended
const NUM_ENDPOINTS: usize = 16;

// Endpoints are named relative to the host, so In is a device TX and Out is a device RX
// +1 for the control endpoint at index 0
static mut EP_IN_WAKERS: [Option<Waker>; NUM_ENDPOINTS + 1] = [const { None }; NUM_ENDPOINTS + 1];
static mut EP_OUT_WAKERS: [Option<Waker>; NUM_ENDPOINTS + 1] = [const { None }; NUM_ENDPOINTS + 1];

// In case a non-aligned buffer is needed, we reserve this one
// We'll warn if it's used
static mut ALIGNED_BUFFER: Aligned<A4, [u8; MAX_FIFO_SIZE as usize]> =
    Aligned::<A4, _>([0; MAX_FIFO_SIZE as usize]);
static ALIGNED_BUFFER_MUTEX: Mutex = Mutex::new();

#[derive(Debug, Clone, Copy, PartialEq)]
enum UsbSpeed {
    Full, // 12Mbps
    High, // 480Mbps
}

impl UsbSpeed {
    fn value(&self) -> pac::mss_usb_device_speed_t {
        match self {
            UsbSpeed::Full => pac::mss_usb_device_speed_t_MSS_USB_DEVICE_FS,
            UsbSpeed::High => pac::mss_usb_device_speed_t_MSS_USB_DEVICE_HS,
        }
    }
}

impl Default for UsbSpeed {
    fn default() -> Self {
        UsbSpeed::High
    }
}

//------------------------------------------------------
// Driver

struct EndpointDetails {
    used: bool,
    fifo_addr: u16,
    fifo_size: u16,
    endpoint_type: EndpointType,
    max_packet_size: u16,
}

impl Default for EndpointDetails {
    fn default() -> Self {
        Self {
            used: false,
            fifo_addr: 0,
            fifo_size: 0,
            endpoint_type: EndpointType::Interrupt,
            max_packet_size: 0,
        }
    }
}

#[derive(Default)]
pub struct UsbDriver<'a> {
    speed: UsbSpeed,
    phantom: core::marker::PhantomData<&'a ()>,
    // Endpoint number 1 maps to index 0
    in_endpoints_allocated: [EndpointDetails; NUM_ENDPOINTS],
    out_endpoints_allocated: [EndpointDetails; NUM_ENDPOINTS],
}

static mut DRIVER_TAKEN: bool = false;
static mut DRIVER_INITIALIZED: bool = false;

impl<'a> Peripheral for UsbDriver<'a> {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if DRIVER_TAKEN {
                None
            } else {
                DRIVER_TAKEN = true;
                Some(Self::default())
            }
        })
    }

    unsafe fn steal() -> Self {
        Self::default()
    }
}

impl<'a> UsbDriver<'a> {
    pub fn set_full_speed(&mut self) {
        self.speed = UsbSpeed::Full;
    }

    fn init(&self) {
        unsafe {
            // Unlike the other peripherals, the USB peripheral needs to be explicitly
            // turned off and on, or else DMA won't work.
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_USB,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_OFF,
            );
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_USB,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
            );
            pac::init_usb_dma_upper_address();
            pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_USB_DMA_INT_OFFSET, 2);
            pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_USB_MC_INT_OFFSET, 2);
            pac::MSS_USBD_CIF_init(self.speed.value());
            pac::MSS_USBD_CIF_cep_configure();
            // MSS_USBD_CIF_dev_connect
            (*pac::USB).POWER |= pac::POWER_REG_SOFT_CONN_MASK as u8;
        }
        log::debug!("USB initialized");
    }
}

impl<'a> embassy_usb_driver::Driver<'a> for UsbDriver<'a> {
    type EndpointOut = EndpointOut<'a>;
    type EndpointIn = EndpointIn<'a>;
    type ControlPipe = ControlPipe<'a>;
    type Bus = UsbBus<'a>;

    fn alloc_endpoint_out(
        &mut self,
        endpoint_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        log::trace!(
            "UsbDriver::alloc_endpoint_out: {:?}, {:?}",
            endpoint_type,
            max_packet_size
        );
        if max_packet_size > MAX_FIFO_SIZE {
            return Err(EndpointAllocError);
        }
        // FIFO size must be a power of 2
        let mut fifo_size = max_packet_size.next_power_of_two();
        if fifo_size < 1024 {
            // If we're using a small enough packet size, we increase the FIFO size
            // to allow for double packet buffering
            fifo_size = (fifo_size + 1).next_power_of_two();
        }
        if let Some(i) = self.out_endpoints_allocated.iter().position(|x| !x.used) {
            let fifo_addr = if i == 0 {
                OUT_ENDPOINTS_START_ADDR
            } else {
                self.out_endpoints_allocated[i - 1].fifo_addr
                    + self.out_endpoints_allocated[i - 1].fifo_size
            };
            if fifo_addr + fifo_size > IN_ENDPOINTS_START_ADDR {
                log::error!(
                    "FIFO address is out of bounds for out endpoint {:?} with size {:?}",
                    i,
                    max_packet_size
                );
                return Err(EndpointAllocError);
            }
            self.out_endpoints_allocated[i] = EndpointDetails {
                used: true,
                fifo_addr: fifo_addr,
                fifo_size: fifo_size,
                endpoint_type: endpoint_type,
                max_packet_size: max_packet_size,
            };
            Ok(EndpointOut {
                phantom: core::marker::PhantomData,
                info: EndpointInfo {
                    addr: EndpointAddress::from_parts(i + 1, Direction::Out),
                    max_packet_size: max_packet_size,
                    interval_ms: interval_ms,
                    ep_type: endpoint_type,
                },
            })
        } else {
            Err(EndpointAllocError)
        }
    }

    fn alloc_endpoint_in(
        &mut self,
        endpoint_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        log::trace!(
            "UsbDriver::alloc_endpoint_in: {:?}, {:?}",
            endpoint_type,
            max_packet_size
        );
        if max_packet_size > MAX_FIFO_SIZE {
            log::error!("Max packet size is greater than the allowed max packet size for this transfer type {} expected for {:?}, got {}", MAX_FIFO_SIZE, endpoint_type, max_packet_size);
            return Err(EndpointAllocError);
        }
        // FIFO size must be a power of 2
        let mut fifo_size = max_packet_size.next_power_of_two();
        if fifo_size < 1024 {
            // If we're using a small enough packet size, we increase the FIFO size
            // to allow for double packet buffering
            fifo_size = (fifo_size + 1).next_power_of_two();
        }
        if let Some(i) = self.in_endpoints_allocated.iter().position(|x| !x.used) {
            let fifo_addr = if i == 0 {
                IN_ENDPOINTS_START_ADDR
            } else {
                self.in_endpoints_allocated[i - 1].fifo_addr
                    + self.in_endpoints_allocated[i - 1].fifo_size
            };
            if fifo_addr.checked_add(fifo_size).is_none() {
                log::error!(
                    "FIFO address is out of bounds for in endpoint {:?} with size {:?}",
                    i,
                    max_packet_size
                );
                return Err(EndpointAllocError);
            }
            self.in_endpoints_allocated[i] = EndpointDetails {
                used: true,
                fifo_addr: fifo_addr,
                fifo_size: fifo_size,
                endpoint_type: endpoint_type,
                max_packet_size: max_packet_size,
            };

            Ok(EndpointIn {
                phantom: core::marker::PhantomData,
                info: EndpointInfo {
                    addr: EndpointAddress::from_parts(i + 1, Direction::In),
                    max_packet_size: max_packet_size,
                    interval_ms: interval_ms,
                    ep_type: endpoint_type,
                },
            })
        } else {
            Err(EndpointAllocError)
        }
    }

    fn start(self, control_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        self.init();
        for (i, ep) in self.in_endpoints_allocated.iter().enumerate() {
            if ep.used {
                configure_endpoint(
                    Direction::In,
                    i + 1,
                    ep.endpoint_type,
                    ep.max_packet_size,
                    ep.fifo_addr,
                    ep.fifo_size,
                    self.speed,
                );
            } else {
                break;
            }
        }
        for (i, ep) in self.out_endpoints_allocated.iter().enumerate() {
            if ep.used {
                configure_endpoint(
                    Direction::Out,
                    i + 1,
                    ep.endpoint_type,
                    ep.max_packet_size,
                    ep.fifo_addr,
                    ep.fifo_size,
                    self.speed,
                );
            } else {
                break;
            }
        }
        critical_section::with(|_| unsafe {
            DRIVER_INITIALIZED = true;

            #[allow(static_mut_refs)]
            for waker in EP_IN_WAKERS.iter() {
                if let Some(waker) = waker {
                    waker.wake_by_ref();
                }
            }
            #[allow(static_mut_refs)]
            for waker in EP_OUT_WAKERS.iter() {
                if let Some(waker) = waker {
                    waker.wake_by_ref();
                }
            }
        });

        (
            UsbBus {
                phantom: core::marker::PhantomData,
            },
            ControlPipe {
                phantom: core::marker::PhantomData,
                max_packet_size: control_packet_size as usize,
            },
        )
    }
}

//------------------------------------------------------
// EndpointOut

// Host -> Device
pub struct EndpointOut<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
    info: EndpointInfo,
}

impl<'a> embassy_usb_driver::EndpointOut for EndpointOut<'a> {
    async fn read(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        todo!("read not implemented")
    }
}

impl<'a> embassy_usb_driver::Endpoint for EndpointOut<'a> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        log::trace!("USB EndpointOut::wait_enabled WAITING");
        let index = self.info.addr.index();
        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                EP_OUT_WAKERS[index] = Some(cx.waker().clone());
            });
            if unsafe { DRIVER_INITIALIZED } {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        log::trace!("USB EndpointOut::wait_enabled OK");
    }
}

//------------------------------------------------------
// EndpointIn

// Device -> Host
pub struct EndpointIn<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
    info: EndpointInfo,
}

impl<'a> embassy_usb_driver::EndpointIn for EndpointIn<'a> {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        log::trace!(
            "USB EndpointIn::write {} : {:?}",
            self.info.addr.index(),
            data
        );
        poll_fn(move |cx| {
            critical_section::with(|_| unsafe {
                EP_IN_WAKERS[self.info.addr.index()] = Some(cx.waker().clone());
                // TODO
                Poll::Pending::<()>
            })
        })
        .await;
        Ok(())
    }
}

impl<'a> embassy_usb_driver::Endpoint for EndpointIn<'a> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        log::trace!("USB EndpointIn::wait_enabled WAITING");
        let index = self.info.addr.index();
        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                EP_IN_WAKERS[index] = Some(cx.waker().clone());
            });
            if unsafe { DRIVER_INITIALIZED } {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        log::trace!("USB EndpointIn::wait_enabled OK");
    }
}

//------------------------------------------------------
// Endpoint Common
//
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

const MAX_FIFO_SIZE: u16 = 4096;
const OUT_ENDPOINTS_START_ADDR: u16 = 0x0000;
const IN_ENDPOINTS_START_ADDR: u16 = 0x8000;

fn endpoint_to_dma_channel(endpoint: usize, direction: Direction) -> pac::mss_usb_dma_channel_t {
    match (endpoint, direction) {
        (1, Direction::Out) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL1,
        (2, Direction::Out) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL2,
        (1, Direction::In) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL3,
        (2, Direction::In) => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL4,
        _ => pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL_NA,
    }
}

fn configure_endpoint(
    direction: Direction,
    endpoint: usize,
    endpoint_type: EndpointType,
    max_packet_size: u16,
    fifo_addr: u16,
    fifo_size: u16,
    speed: UsbSpeed,
) {
    unsafe {
        // Get the standard max packet size based on speed and transfer type
        let std_max_pkt_sz = if speed == UsbSpeed::High {
            match endpoint_type {
                EndpointType::Bulk => pac::USB_HS_BULK_MAX_PKT_SIZE,
                EndpointType::Interrupt => pac::USB_HS_INTERRUPT_MAX_PKT_SIZE,
                EndpointType::Isochronous => pac::USB_HS_ISO_MAX_PKT_SIZE,
                EndpointType::Control => pac::USB_HS_BULK_MAX_PKT_SIZE, // Control uses same as bulk
            }
        } else {
            match endpoint_type {
                EndpointType::Bulk => pac::USB_FS_BULK_MAX_PKT_SIZE,
                EndpointType::Interrupt => pac::USB_FS_INTERRUPT_MAX_PKT_SIZE,
                EndpointType::Isochronous => pac::USB_FS_ISO_MAX_PKT_SIZE,
                EndpointType::Control => pac::USB_FS_BULK_MAX_PKT_SIZE,
            }
        };

        if max_packet_size > std_max_pkt_sz as u16 {
            panic!("Max packet size is greater than the allowed max packet size for this transfer type {} expected for {:?}, got {}", std_max_pkt_sz, endpoint_type, max_packet_size);
        }

        let dma_channel = endpoint_to_dma_channel(endpoint, direction);
        let dma_enable = dma_channel != pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL_NA;
        // Configure double packet buffering if there's enough FIFO space reserved
        let dpb_enable = max_packet_size * 2 <= fifo_size;

        // TODO: I think we need to save this for reuse
        let mut ep = pac::mss_usb_ep_t {
            num: endpoint as u32,
            xfr_type: transfer_type(endpoint_type),
            max_pkt_size: max_packet_size,
            fifo_size,
            fifo_addr,
            dma_enable: dma_enable as u8,
            dpb_enable: dpb_enable as u8,
            dma_channel,
            ..default_ep()
        };

        if direction == Direction::Out {
            pac::MSS_USBD_CIF_rx_ep_configure(&mut ep);
        } else {
            pac::MSS_USBD_CIF_tx_ep_configure(&mut ep);
        }
    }
}

fn transfer_type(endpoint_type: EndpointType) -> pac::mss_usb_xfr_type_t {
    match endpoint_type {
        EndpointType::Bulk => pac::mss_usb_xfr_type_t_MSS_USB_XFR_BULK,
        EndpointType::Interrupt => pac::mss_usb_xfr_type_t_MSS_USB_XFR_INTERRUPT,
        EndpointType::Isochronous => pac::mss_usb_xfr_type_t_MSS_USB_XFR_ISO,
        EndpointType::Control => pac::mss_usb_xfr_type_t_MSS_USB_XFR_BULK,
    }
}

//------------------------------------------------------
// ControlPipe

struct ControlEvent {
    setup: bool,
    data_out: bool,
    data_in: bool,
    status: bool,
}

static mut CONTROL_EVENTS: ControlEvent = ControlEvent {
    setup: false,
    data_out: false,
    data_in: false,
    status: false,
};

pub struct ControlPipe<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
    max_packet_size: usize,
}

impl<'a> embassy_usb_driver::ControlPipe for ControlPipe<'a> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size
    }

    async fn setup(&mut self) -> [u8; 8] {
        log::trace!("USB ControlPipe::setup");
        loop {
            poll_fn(move |cx| {
                critical_section::with(|_| unsafe {
                    EP_OUT_WAKERS[0] = Some(cx.waker().clone());
                    if CONTROL_EVENTS.setup && pac::MSS_USB_CIF_cep_is_rxpktrdy() != 0 {
                        CONTROL_EVENTS.setup = false;
                        Poll::Ready(())
                    } else {
                        Poll::Pending
                    }
                })
            })
            .await;

            let mut setup_packet = [0; 8];
            read_control_packet(&mut setup_packet, self.max_packet_size);
            // For some reason, sometimes the packet wasn't actually ready.
            // If we loop here and wait for the next interrupt, we seem to get the right packet.
            if setup_packet != [0; 8] {
                log::trace!("USB ControlPipe::setup packet: {:x?}", setup_packet);
                return setup_packet;
            }
        }
    }

    // Host -> Device
    // If a setup packet is received while this is waiting, this must return `EndpointError::Disabled`
    // TODO: Is there actually a point where we can do this?
    async fn data_out(
        &mut self,
        buf: &mut [u8],
        first: bool,
        last: bool,
    ) -> Result<usize, EndpointError> {
        todo!("data_out not implemented")

        // It does not seem like we need to do anything special when last is true
    }

    // Device -> Host
    // If a setup packet is received while this is waiting, this must return `EndpointError::Disabled`
    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        log::trace!(
            "USB ControlPipe::data_in: {:x?}, {:?}, {:?}",
            data,
            first,
            last
        );

        let mut aligned_buffer = data;
        let mut lock_token = None;

        #[allow(static_mut_refs)]
        unsafe {
            if data.as_ptr().align_offset(4) != 0 {
                log::warn!("Endpoint data should be 32-bit aligned");
                lock_token = Some(ALIGNED_BUFFER_MUTEX.lock());
                ALIGNED_BUFFER[..data.len()].copy_from_slice(data);
                aligned_buffer = &ALIGNED_BUFFER[..data.len()];
            }
            let mut ep = default_ep();
            ep.num = 0;
            ep.buf_addr = aligned_buffer.as_ptr() as *mut u8;
            ep.txn_length = aligned_buffer.len() as u32;
            ep.max_pkt_size = self.max_packet_size as u16;
            // When last is true, we send a zero length packet after the transfer
            // This means that xfr_count == xfr_length
            // Otherwise, xfr_count < xfr_length
            // The specifics of these numbers are otherwise not important
            ep.xfr_count = 0;
            ep.xfr_length = if last { 0 } else { 1 };
            pac::MSS_USBD_CIF_cep_write_pkt(&mut ep);
        }

        if let Some(lock_token) = lock_token {
            ALIGNED_BUFFER_MUTEX.release(lock_token);
        }
        Ok(())
    }

    async fn accept(&mut self) {
        unsafe {
            // MSS_USBD_CIF_cep_end_zdr()
            (*pac::USB).INDEXED_CSR.DEVICE_EP0.CSR0 =
                (pac::CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK | pac::CSR0L_DEV_DATA_END_MASK) as u16;
        }
        log::trace!("USB ControlPipe::accept");
    }

    async fn reject(&mut self) {
        todo!("reject not implemented")
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        // Unless we wait here, the zero length packet is not sent
        // See: mss_usb_device.c:mss_usbd_cep_setup_cb
        for _ in 0..5000 {
            core::hint::spin_loop();
        }

        unsafe {
            ((*pac::USB).FADDR) = addr;
        }
        log::trace!("USB ControlPipe::accept_set_address: {}", addr);
    }
}

fn read_control_packet(buf: &mut [u8], max_pkt_size: usize) {
    unsafe {
        let mut ep = pac::mss_usb_ep_t {
            num: 0,
            buf_addr: buf.as_mut_ptr() as *mut u8,
            txn_length: buf.len() as u32,
            xfr_length: buf.len() as u32,
            max_pkt_size: max_pkt_size as u16,
            xfr_type: pac::mss_usb_xfr_type_t_MSS_USB_XFR_CONTROL,
            state: pac::mss_usb_ep_state_t_MSS_USB_CEP_RX,

            ..default_ep()
        };
        pac::MSS_USBD_CIF_cep_read_pkt(&mut ep);
    }
}

const fn default_ep() -> pac::mss_usb_ep_t {
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

        // Unsued?
        cep_cmd_addr: core::ptr::null_mut(),
        cep_data_dir: 0,
        disable_ping: 0,
        interval: 0,
        stall: 0,
        req_pkt_n: 0,
        tdev_idx: 0,
    }
}

//------------------------------------------------------
// Bus

struct BusEvent {
    reset: bool,
    suspend: bool,
    resume: bool,
    disconnect: bool,
}

static mut BUS_EVENT_WAKER: Option<Waker> = None;
static mut BUS_DISCONNECT_WAKER: Option<Waker> = None;

static mut BUS_EVENT: BusEvent = BusEvent {
    reset: false,
    suspend: false,
    resume: false,
    disconnect: false,
};

pub struct UsbBus<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> UsbBus<'a> {
    pub async fn await_disconnect(&self) {
        poll_fn(move |cx| {
            critical_section::with(|_| unsafe {
                BUS_DISCONNECT_WAKER = Some(cx.waker().clone());
                if BUS_EVENT.disconnect {
                    BUS_EVENT.disconnect = false;
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }
}

impl<'a> embassy_usb_driver::Bus for UsbBus<'a> {
    async fn poll(&mut self) -> Event {
        let ret = poll_fn(move |cx| {
            critical_section::with(|_| unsafe {
                BUS_EVENT_WAKER = Some(cx.waker().clone());
                // The order here matters since a host may send a suspend and reset
                // quick succession, and we want to handle the reset first
                if BUS_EVENT.suspend {
                    BUS_EVENT.suspend = false;
                    Poll::Ready(Event::Suspend)
                } else if BUS_EVENT.reset {
                    BUS_EVENT.reset = false;
                    Poll::Ready(Event::Reset)
                } else if BUS_EVENT.resume {
                    BUS_EVENT.resume = false;
                    Poll::Ready(Event::Resume)
                } else {
                    Poll::Pending
                }
            })
        })
        .await;
        log::debug!("USB poll: {:?}", ret);
        if ret == Event::Reset {
            unsafe {
                pac::MSS_USBD_CIF_cep_configure();
            }
        }
        ret
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        log::trace!(
            "USB Bus::endpoint_set_enabled: {:?}{}, {:?}",
            ep_addr.direction(),
            ep_addr.index(),
            enabled
        );
        todo!("endpoint_set_enabled not implemented")
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        todo!("endpoint_set_stalled not implemented")
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        todo!("endpoint_is_stalled not implemented")
    }

    async fn enable(&mut self) {}
    async fn disable(&mut self) {}

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        // TODO: Does the MPFS support this? The platform code seems to hint at its existence,
        // yet I don't see any way to actually create a remote wakeup signal.
        Err(embassy_usb_driver::Unsupported)
    }
}

//------------------------------------------------------
// MSS USB CIF Callbacks

#[no_mangle]
#[doc(hidden)]
#[allow(non_upper_case_globals)]
pub static g_mss_usbd_cb: pac::mss_usbd_cb_t = pac::mss_usbd_cb_t {
    usbd_ep_rx: Some(usbd_ep_rx),
    usbd_ep_tx_complete: Some(usbd_ep_tx_complete),
    usbd_cep_setup: Some(usbd_cep_setup),
    usbd_cep_rx: Some(usbd_cep_rx),
    usbd_cep_tx_complete: Some(usbd_cep_tx_complete),
    usbd_sof: Some(usbd_sof),
    usbd_reset: Some(usbd_reset),
    usbd_suspend: Some(usbd_suspend),
    usbd_resume: Some(usbd_resume),
    usbd_disconnect: Some(usbd_disconnect),
    usbd_dma_handler: Some(usbd_dma_handler),
};

extern "C" fn usbd_ep_rx(num: pac::mss_usb_ep_num_t, status: u8) {
    // TODO: Implement endpoint receive callback
    log::trace!("usbd_ep_rx: {:?}, {:?}", num, status);
}

extern "C" fn usbd_ep_tx_complete(num: pac::mss_usb_ep_num_t, status: u8) {
    // TODO: Implement endpoint transmit complete callback
    log::trace!("usbd_ep_tx_complete: {:?}, {:?}", num, status);
}

extern "C" fn usbd_cep_setup(status: u8) {
    log::trace!("usbd_cep_setup: {:?}", status);
    // This is called in three cases:
    // 1. When the control endpoint is stalled
    // 2. When the control endpoint recieves a setup packet before the setup has completed
    // 3. When the control endpoint recieves a normal setup packet
    //
    // The CIF clears the PHY of the stall in the first case already, so there's nothing left to do
    // The other two we can handle the same (I think?)
    critical_section::with(|_| unsafe {
        CONTROL_EVENTS.setup = true;
        #[allow(static_mut_refs)]
        if let Some(waker) = EP_OUT_WAKERS[0].as_mut() {
            waker.wake_by_ref();
        }
        if let Some(waker) = EP_IN_WAKERS[0].as_mut() {
            waker.wake_by_ref();
        }
    });
}

extern "C" fn usbd_cep_rx(status: u8) {
    // TODO: Implement control endpoint receive callback
    log::trace!("usbd_cep_rx: {:?}", status);
}

extern "C" fn usbd_cep_tx_complete(status: u8) {
    // TODO: Implement control endpoint transmit complete callback
    log::trace!("usbd_cep_tx_complete: {:?}", status);
}

extern "C" fn usbd_sof(status: u8) {
    // TODO: Implement start of frame callback
    log::trace!("usbd_sof: {:?}", status);
}

extern "C" fn usbd_reset() {
    log::trace!("usbd_reset");
    critical_section::with(|_| unsafe {
        BUS_EVENT.reset = true;
        #[allow(static_mut_refs)]
        if let Some(waker) = BUS_EVENT_WAKER.as_mut() {
            waker.wake_by_ref();
        }
    });
}

extern "C" fn usbd_suspend() {
    log::trace!("usbd_suspend");
    critical_section::with(|_| unsafe {
        BUS_EVENT.suspend = true;
        #[allow(static_mut_refs)]
        if let Some(waker) = BUS_EVENT_WAKER.as_mut() {
            waker.wake_by_ref();
        }
    });
}

extern "C" fn usbd_resume() {
    log::trace!("usbd_resume");
    critical_section::with(|_| unsafe {
        BUS_EVENT.resume = true;
        #[allow(static_mut_refs)]
        if let Some(waker) = BUS_EVENT_WAKER.as_mut() {
            waker.wake_by_ref();
        }
    });
}

extern "C" fn usbd_disconnect() {
    log::trace!("usbd_disconnect");
    critical_section::with(|_| unsafe {
        BUS_EVENT.disconnect = true;
        #[allow(static_mut_refs)]
        if let Some(waker) = BUS_DISCONNECT_WAKER.as_mut() {
            waker.wake_by_ref();
        }
    });
}

extern "C" fn usbd_dma_handler(
    ep_num: pac::mss_usb_ep_num_t,
    dma_dir: pac::mss_usb_dma_dir_t,
    status: u8,
    dma_addr_val: u32,
) {
    // TODO: Implement DMA handler callback
    log::trace!(
        "usbd_dma_handler: {:?}, {:?}, {:?}, {:?}",
        ep_num,
        dma_dir,
        status,
        dma_addr_val
    );
}
