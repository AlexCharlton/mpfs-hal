use embassy_usb_driver::host::{
    channel, ChannelError, HostError, SetupPacket, UsbChannel, UsbHostDriver,
};
use embassy_usb_driver::EndpointInfo;

use crate::{pac, Peripheral};

#[derive(Default)]
pub struct UsbHost {}

static mut HOST_TAKEN: bool = false;

impl Peripheral for UsbHost {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if HOST_TAKEN {
                None
            } else {
                HOST_TAKEN = true;
                Some(Self::default())
            }
        })
    }

    unsafe fn steal() -> Self {
        Self::default()
    }
}

impl UsbHost {
    pub fn start(&self) {
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

            // Not used (needed) in Device, but called in example
            // PLIC_EnableIRQ(USB_DMA_PLIC);
            // PLIC_EnableIRQ(USB_MC_PLIC);

            //  called in MSS_USBH_init(&MSS_USBH_user_cb);
            pac::MSS_USBH_CIF_init();
            // MSS_USBH_configure_control_pipe(TDEV_R);

            // MSS_USBH_HID_init doesn't call any underlying functions
        }
        // This is called in mss-usb/mpfs-usb-host-hid/src/application/hart1/u54_1.c
        //  MSS_USBH_register_class_driver(MSS_USBH_HID_get_handle());

        // These are called in a systick handler
        //     MSS_USBH_task();
        //     MSS_USBH_HID_task();
        //     MSS_USBH_1ms_tick();
    }
}

impl UsbHostDriver for UsbHost {
    type Channel<T: channel::Type, D: channel::Direction> = Channel<T, D>;

    async fn wait_for_device_event(&self) -> embassy_usb_driver::host::DeviceEvent {
        todo!()
    }

    async fn bus_reset(&self) {
        todo!()
    }

    // `pre` - device is low-speed and communication is going through hub, so send PRE packet
    // What does this mean?
    fn alloc_channel<T: channel::Type, D: channel::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        pre: bool,
    ) -> Result<Self::Channel<T, D>, HostError> {
        todo!()
    }
}

//------------------------------------------------------
pub struct Channel<T: channel::Type, D: channel::Direction> {
    pub direction: D,
    pub channel_type: T,
}

impl<T: channel::Type, D: channel::Direction> UsbChannel<T, D> for Channel<T, D> {
    async fn control_in(
        &mut self,
        setup: &SetupPacket,
        buf: &mut [u8],
    ) -> Result<usize, ChannelError>
    where
        T: channel::IsControl,
        D: channel::IsIn,
    {
        todo!()
    }

    async fn control_out(&mut self, setup: &SetupPacket, buf: &[u8]) -> Result<usize, ChannelError>
    where
        T: channel::IsControl,
        D: channel::IsOut,
    {
        todo!()
    }

    fn retarget_channel(
        &mut self,
        addr: u8,
        endpoint: &EndpointInfo,
        pre: bool,
    ) -> Result<(), HostError> {
        todo!()
    }
    /// Send IN request of type other from control
    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsIn,
    {
        todo!()
    }
    /// Send OUT request of type other from control
    async fn request_out(&mut self, buf: &[u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsOut,
    {
        todo!()
    }
}
