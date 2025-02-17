use crate::Peripheral;
use mpfs_pac as pac;

use embassy_usb_driver::{
    EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event,
};

//------------------------------------------------------
// Driver

pub struct UsbDriver<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

static mut DRIVER_TAKEN: bool = false;

impl<'a> Peripheral for UsbDriver<'a> {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if DRIVER_TAKEN {
                None
            } else {
                DRIVER_TAKEN = true;
                init_usb();
                Some(Self {
                    phantom: core::marker::PhantomData,
                })
            }
        })
    }

    unsafe fn steal() -> Self {
        Self {
            phantom: core::marker::PhantomData,
        }
    }
}

fn init_usb() {
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
        // TODO: Make this configurable
        pac::MSS_USBD_CIF_init(pac::mss_usb_device_speed_t_MSS_USB_DEVICE_FS);
        // MSS_USBD_CIF_dev_connect
        (*pac::USB).POWER |= pac::POWER_REG_SOFT_CONN_MASK as u8;
    }
    log::debug!("USB initialized");
}

impl<'a> embassy_usb_driver::Driver<'a> for UsbDriver<'a> {
    type EndpointOut = EndpointOut<'a>;
    type EndpointIn = EndpointIn<'a>;
    type ControlPipe = ControlPipe<'a>;
    type Bus = UsbBus<'a>;

    fn alloc_endpoint_out(
        &mut self,
        _endpoint_type: EndpointType,
        _max_packet_size: u16,
        _interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        todo!()
    }

    fn alloc_endpoint_in(
        &mut self,
        _endpoint_type: EndpointType,
        _max_packet_size: u16,
        _interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        todo!()
    }

    fn start(self, _control_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        todo!()
    }
}

//------------------------------------------------------
// EndpointOut

pub struct EndpointOut<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> embassy_usb_driver::EndpointOut for EndpointOut<'a> {
    async fn read(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        todo!()
    }
}

impl<'a> embassy_usb_driver::Endpoint for EndpointOut<'a> {
    fn info(&self) -> &EndpointInfo {
        todo!()
    }

    async fn wait_enabled(&mut self) {
        todo!()
    }
}

//------------------------------------------------------
// EndpointIn

pub struct EndpointIn<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> embassy_usb_driver::EndpointIn for EndpointIn<'a> {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        todo!()
    }
}

impl<'a> embassy_usb_driver::Endpoint for EndpointIn<'a> {
    fn info(&self) -> &EndpointInfo {
        todo!()
    }

    async fn wait_enabled(&mut self) {
        todo!()
    }
}

//------------------------------------------------------
// ControlPipe

pub struct ControlPipe<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> embassy_usb_driver::ControlPipe for ControlPipe<'a> {
    fn max_packet_size(&self) -> usize {
        todo!()
    }

    async fn setup(&mut self) -> [u8; 8] {
        todo!()
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        first: bool,
        last: bool,
    ) -> Result<usize, EndpointError> {
        todo!()
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        todo!()
    }

    async fn accept(&mut self) {
        todo!()
    }

    async fn reject(&mut self) {
        todo!()
    }

    async fn accept_set_address(&mut self, addr: u8) {
        todo!()
    }
}

//------------------------------------------------------
// Bus

pub struct UsbBus<'a> {
    phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a> embassy_usb_driver::Bus for UsbBus<'a> {
    async fn enable(&mut self) {
        todo!()
    }

    async fn disable(&mut self) {
        todo!()
    }

    async fn poll(&mut self) -> Event {
        todo!()
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        todo!()
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        todo!()
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        todo!()
    }

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        todo!()
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
