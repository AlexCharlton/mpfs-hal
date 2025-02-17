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
