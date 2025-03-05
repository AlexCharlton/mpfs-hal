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
        // TODO
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

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsIn,
    {
        todo!()
    }

    async fn request_out(&mut self, buf: &[u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsOut,
    {
        todo!()
    }
}
