use crate::Peripheral;
use mpfs_pac as pac;

use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType,
    Event,
};

// This should not be raised unless related constants in Endpoint Common are ammended
const NUM_ENDPOINTS: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsbSpeed {
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
        UsbSpeed::Full
    }
}

//------------------------------------------------------
// Driver

#[derive(Default)]
struct EndpointDetails {
    used: bool,
    fifo_addr: u16,
    fifo_size: u16,
}

#[derive(Default)]
pub struct UsbDriver<'a> {
    initialized: bool,
    speed: UsbSpeed,
    phantom: core::marker::PhantomData<&'a ()>,
    in_endpoints_allocated: [EndpointDetails; NUM_ENDPOINTS],
    out_endpoints_allocated: [EndpointDetails; NUM_ENDPOINTS],
}

static mut DRIVER_TAKEN: bool = false;

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
    pub fn init(&mut self, speed: UsbSpeed) {
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
            pac::MSS_USBD_CIF_init(speed.value());
            // MSS_USBD_CIF_dev_connect
            (*pac::USB).POWER |= pac::POWER_REG_SOFT_CONN_MASK as u8;
        }
        self.initialized = true;
        self.speed = speed;
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
            "alloc_endpoint_out: {:?}, {:?}",
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
            };
            Ok(EndpointOut::new(
                i,
                endpoint_type,
                max_packet_size,
                interval_ms,
                fifo_addr,
                fifo_size,
                self.speed,
            ))
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
            "alloc_endpoint_in: {:?}, {:?}",
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
            };

            Ok(EndpointIn::new(
                i,
                endpoint_type,
                max_packet_size,
                interval_ms,
                fifo_addr,
                fifo_size,
                self.speed,
            ))
        } else {
            Err(EndpointAllocError)
        }
    }

    fn start(self, _control_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        todo!("start not implemented")
    }
}

//------------------------------------------------------
// EndpointOut

pub struct EndpointOut<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
    info: EndpointInfo,
}

impl<'a> EndpointOut<'a> {
    pub fn new(
        i: usize,
        endpoint_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
        fifo_addr: u16,
        fifo_size: u16,
        speed: UsbSpeed,
    ) -> Self {
        configure_endpoint(
            Direction::Out,
            i,
            endpoint_type,
            max_packet_size,
            fifo_addr,
            fifo_size,
            speed,
        );

        Self {
            phantom: core::marker::PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(i, Direction::Out),
                max_packet_size: max_packet_size,
                interval_ms: interval_ms,
                ep_type: endpoint_type,
            },
        }
    }
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
        // No need to wait, the endpoint is enabled by default
    }
}

//------------------------------------------------------
// EndpointIn

pub struct EndpointIn<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
    info: EndpointInfo,
}

impl<'a> EndpointIn<'a> {
    pub fn new(
        i: usize,
        endpoint_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
        fifo_addr: u16,
        fifo_size: u16,
        speed: UsbSpeed,
    ) -> Self {
        configure_endpoint(
            Direction::In,
            i,
            endpoint_type,
            max_packet_size,
            fifo_addr,
            fifo_size,
            speed,
        );
        Self {
            phantom: core::marker::PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(i, Direction::In),
                max_packet_size: max_packet_size,
                interval_ms: interval_ms,
                ep_type: endpoint_type,
            },
        }
    }
}

impl<'a> embassy_usb_driver::EndpointIn for EndpointIn<'a> {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        todo!("write not implemented")
    }
}

impl<'a> embassy_usb_driver::Endpoint for EndpointIn<'a> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        // No need to wait, the endpoint is enabled by default
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

        let mut ep = pac::mss_usb_ep_t {
            num: endpoint as u32,
            xfr_type: transfer_type(endpoint_type),
            max_pkt_size: max_packet_size,
            fifo_size,
            fifo_addr,
            dma_enable: dma_enable as u8,
            dpb_enable: dpb_enable as u8,
            dma_channel,
            add_zlp: 0,     // Should this be configurable?
            num_usb_pkt: 1, // This must always be 1, according to mss_usb_device.h

            // Not used for configuration
            state: pac::mss_usb_ep_state_t_MSS_USB_EP_VALID,
            buf_addr: core::ptr::null_mut(),
            cep_cmd_addr: core::ptr::null_mut(),
            cep_data_dir: 0,
            disable_ping: 0,
            interval: 0,
            stall: 0,
            req_pkt_n: 0,
            tdev_idx: 0,
            txn_count: 0,
            txn_length: 0,
            xfr_count: 0,
            xfr_length: 0,
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

pub struct ControlPipe<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> embassy_usb_driver::ControlPipe for ControlPipe<'a> {
    fn max_packet_size(&self) -> usize {
        todo!("max_packet_size not implemented")
    }

    async fn setup(&mut self) -> [u8; 8] {
        todo!("setup not implemented")
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        first: bool,
        last: bool,
    ) -> Result<usize, EndpointError> {
        todo!("data_out not implemented")
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        todo!("data_in not implemented")
    }

    async fn accept(&mut self) {
        todo!("accept not implemented")
    }

    async fn reject(&mut self) {
        todo!("reject not implemented")
    }

    async fn accept_set_address(&mut self, addr: u8) {
        todo!("accept_set_address not implemented")
    }
}

//------------------------------------------------------
// Bus

pub struct UsbBus<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> embassy_usb_driver::Bus for UsbBus<'a> {
    async fn enable(&mut self) {
        todo!("enable not implemented")
    }

    async fn disable(&mut self) {
        todo!("disable not implemented")
    }

    async fn poll(&mut self) -> Event {
        todo!("poll not implemented")
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        todo!("endpoint_set_enabled not implemented")
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        todo!("endpoint_set_stalled not implemented")
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        todo!("endpoint_is_stalled not implemented")
    }

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        todo!("remote_wakeup not implemented")
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
    // TODO: Implement control endpoint setup callback
    log::trace!("usbd_cep_setup: {:?}", status);
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
    // TODO: Implement USB reset callback
    log::trace!("usbd_reset");
}

extern "C" fn usbd_suspend() {
    // TODO: Implement USB suspend callback
    log::trace!("usbd_suspend");
}

extern "C" fn usbd_resume() {
    // TODO: Implement USB resume callback
    log::trace!("usbd_resume");
}

extern "C" fn usbd_disconnect() {
    // TODO: Implement USB disconnect callback
    log::trace!("usbd_disconnect");
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
