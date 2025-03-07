use core::future::poll_fn;
use core::task::{Poll, Waker};
use embassy_futures::{select, select::Either};
use embassy_time::{Duration, Timer};
use embassy_usb_driver::host::{
    channel, ChannelError, DeviceEvent, HostError, SetupPacket, UsbChannel, UsbHostDriver,
};
use embassy_usb_driver::{EndpointInfo, EndpointType, Speed};

use mpfs_hal::{pac, Peripheral};

use super::common::*;

// We only have one USB peripheral, so we only need one set of controllers
static mut EP_IN_CONTROLLER: [Option<EndpointController>; NUM_ENDPOINTS + 1] =
    [const { None }; NUM_ENDPOINTS + 1];
static mut EP_OUT_CONTROLLER: [Option<EndpointController>; NUM_ENDPOINTS + 1] =
    [const { None }; NUM_ENDPOINTS + 1];

//------------------------------------------------------

// There's only one USB peripheral, so only one device can be connected at a time
static mut CONNECTED: DeviceEvent = DeviceEvent::Disconnected;
static mut DEVICE_EVENT: Option<DeviceEvent> = None;
static mut DEVICE_EVENT_WAKER: Option<Waker> = None;

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
        loop {
            #[allow(static_mut_refs)]
            unsafe {
                if CONNECTED == DeviceEvent::Disconnected {
                    // When not connected, try to detect connection
                    let session_future = async {
                        // Set session bit periodically to check to see if device is connected
                        (*pac::USB).DEV_CTRL |= pac::DEV_CTRL_SESSION_MASK as u8;
                        Timer::after(Duration::from_millis(5000)).await;
                    };

                    let event_future = poll_fn(|cx| {
                        critical_section::with(|_| {
                            if let Some(DeviceEvent::Connected(_)) = DEVICE_EVENT {
                                let event = DEVICE_EVENT.take().unwrap();
                                CONNECTED = event;
                                Poll::Ready(event)
                            } else {
                                DEVICE_EVENT_WAKER = Some(cx.waker().clone());
                                Poll::Pending
                            }
                        })
                    });

                    let event = select::select(session_future, event_future).await;
                    if let Either::Second(event) = event {
                        return event;
                    }
                } else {
                    // When connected, wait for disconnect
                    poll_fn(|cx| {
                        critical_section::with(|_| {
                            DEVICE_EVENT_WAKER = Some(cx.waker().clone());
                            if let Some(DeviceEvent::Disconnected) = DEVICE_EVENT {
                                DEVICE_EVENT = None;
                                CONNECTED = DeviceEvent::Disconnected;
                                Poll::Ready(())
                            } else {
                                Poll::Pending
                            }
                        })
                    })
                    .await;
                    return embassy_usb_driver::host::DeviceEvent::Disconnected;
                }
            }
        }
    }

    async fn bus_reset(&self) {
        unsafe {
            // MSS_USBH_CIF_assert_bus_reset
            (*pac::USB).POWER |= pac::POWER_REG_BUS_RESET_SIGNAL_MASK as u8;
            Timer::after(Duration::from_millis(50)).await;
            // MSS_USBH_CIF_clr_bus_reset
            (*pac::USB).POWER &= !pac::POWER_REG_BUS_RESET_SIGNAL_MASK as u8;
        }
    }

    fn alloc_channel<T: channel::Type, D: channel::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        pre: bool,
    ) -> Result<Self::Channel<T, D>, HostError> {
        assert!(!pre, "Low speed devices aren't supported");
        log::trace!("alloc_channel {} {:?}", addr, endpoint);
        // TODO We should only ever allocate channels beloging to one address at a time

        if T::ep_type() == EndpointType::Control {
            let mut ep = default_ep();
            ep.num = 0;
            ep.xfr_type = pac::mss_usb_xfr_type_t_MSS_USB_XFR_CONTROL;
            ep.fifo_size = 64;
            ep.max_pkt_size = endpoint.max_packet_size;
            ep.interval = 32768; // Max value
            ep.num_usb_pkt = 1;
            unsafe {
                EP_IN_CONTROLLER[0] = Some({
                    EndpointController {
                        ep,
                        ..EndpointController::default()
                    }
                });
                EP_OUT_CONTROLLER[0] = Some({
                    EndpointController {
                        ep,
                        ..EndpointController::default()
                    }
                });
            }
            Ok(Channel {
                index: 0,
                phantom: core::marker::PhantomData,
            })
        } else {
            todo!("alloc_channel non-control")
        }
    }
}

//------------------------------------------------------

pub struct Channel<T: channel::Type, D: channel::Direction> {
    phantom: core::marker::PhantomData<(T, D)>,
    index: usize,
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
        log::trace!("control_in");
        unsafe {
            let mut aligned_buffer = buf.as_ptr();
            if aligned_buffer.align_offset(4) != 0 {
                let e = EP_IN_CONTROLLER[0].as_mut().unwrap();
                log::warn!("control_in: data should be 32-bit aligned");
                aligned_buffer = e.buffer_addr()[..buf.len()].as_ptr();
            }
            let ep = EP_IN_CONTROLLER[0].as_mut().unwrap();
            ep.ep.buf_addr = aligned_buffer as *mut u8;
            ep.ep.xfr_length = buf.len() as u32;
            ep.ep.xfr_count = 0;
            ep.ep.txn_count = 0;
            ep.ep.xfr_type = pac::mss_usb_xfr_type_t_MSS_USB_XFR_CONTROL;

            critical_section::with(|_| {
                EP_IN_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Setup;
            });

            pac::MSS_USBH_CIF_load_tx_fifo(
                0,
                // Could this cause alignment issues?
                setup.as_bytes().as_ptr() as *mut core::ffi::c_void,
                8,
            );
            // MSS_USBH_CIF_cep_set_setuppktrdy
            (*pac::USB).ENDPOINT[0].TX_CSR |=
                (pac::CSR0L_HOST_SETUP_PKT_MASK | pac::CSR0L_HOST_TX_PKT_RDY_MASK) as u16;
        }

        let read_size = poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                if let EndpointState::RxComplete(i) = EP_IN_CONTROLLER[0].as_ref().unwrap().state {
                    EP_IN_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Idle;
                    Poll::Ready(i)
                } else {
                    EP_IN_CONTROLLER[0].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;

        log::trace!("control_in: read={:x?}", &buf[..read_size]);

        Ok(read_size)
    }

    async fn control_out(&mut self, setup: &SetupPacket, buf: &[u8]) -> Result<usize, ChannelError>
    where
        T: channel::IsControl,
        D: channel::IsOut,
    {
        log::trace!("control_out: setup={:?}, buf={:x?}", setup, buf);
        todo!("control_out")
    }

    fn retarget_channel(
        &mut self,
        addr: u8,
        endpoint: &EndpointInfo,
        pre: bool,
    ) -> Result<(), HostError> {
        assert!(!pre, "Low speed devices aren't supported");
        log::trace!("retarget_channel {} {:?}", addr, endpoint);
        if T::ep_type() == EndpointType::Control {
            unsafe {
                let ep = EP_IN_CONTROLLER[0].as_mut().unwrap();
                pac::MSS_USBH_CIF_cep_configure(&mut ep.ep);
                // pac::MSS_USBH_CIF_cep_set_type0_reg
                (*pac::USB).ENDPOINT[0].TX_TYPE =
                    (mss_speed() << pac::TYPE0_HOST_MP_TARGET_SPEED_SHIFT) as u8;
                // pac::MSS_USBH_CIF_tx_ep_set_target_func_addr(0, addr);
                (*pac::USB).TAR[0].TX_FUNC_ADDR = (addr & 0x7F) as u8;
            }
            Ok(())
        } else {
            todo!("retarget_channel non-control")
        }
    }
    /// Send IN request of type other from control
    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsIn,
    {
        log::trace!("request_in");
        todo!("request_in")
    }
    /// Send OUT request of type other from control
    async fn request_out(&mut self, buf: &[u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsOut,
    {
        log::trace!("request_out");
        todo!("request_out")
    }
}

fn mss_speed() -> pac::mss_usb_device_speed_t {
    let speed = unsafe {
        match CONNECTED {
            DeviceEvent::Connected(speed) => speed,
            DeviceEvent::Disconnected => Speed::Low,
        }
    };
    speed_to_mss_value(speed)
}

//------------------------------------------------------
// MSS USB CIF Callbacks

#[no_mangle]
#[doc(hidden)]
#[allow(non_upper_case_globals)]
pub static g_mss_usbh_cb: pac::mss_usbh_cb_t = pac::mss_usbh_cb_t {
    usbh_cep: Some(usbh_cep),
    usbh_tx_complete: Some(usbh_tx_complete),
    usbh_rx: Some(usbh_rx),
    usbh_dma_handler: Some(usbh_dma_handler),
    usbh_connect: Some(usbh_connect),
    usbh_disconnect: Some(usbh_disconnect),

    usbh_sof: None,             // Never called
    usbh_vbus_error: None,      // Never called
    usbh_babble_error: None,    // Never called
    usbh_session_request: None, // Never called
};

extern "C" fn usbh_cep(status: u8) {
    log::trace!("usbh_cep: status={}", status);
    if status != pac::mss_usb_ep_state_t_MSS_USB_EP_TXN_SUCCESS as u8 {
        log::error!("usbh_cep: non-success status={}", status);
    }
    critical_section::with(|_| unsafe {
        // let out_state = &mut EP_OUT_CONTROLLER[0].as_mut().unwrap().state;
        let in_ep = &mut EP_IN_CONTROLLER[0].as_mut().unwrap();
        let out_ep = &mut EP_OUT_CONTROLLER[0].as_mut().unwrap();
        if in_ep.state == EndpointState::Setup {
            in_ep.state = EndpointState::Rx;
            // MSS_USBH_CIF_cep_set_request_in_pkt
            (*pac::USB).ENDPOINT[0].TX_CSR |= pac::CSR0L_HOST_IN_PKT_REQ_MASK as u16;

        // TODO? When transfer length is 0:
        // MSS_USBH_CIF_cep_set_statuspktrdy_after_out();
        } else if in_ep.state == EndpointState::Rx {
            pac::MSS_USBH_CIF_cep_read_pkt(&mut in_ep.ep);
            if in_ep.ep.xfr_count == in_ep.ep.xfr_length {
                // Done

                // MSS_USBH_CIF_cep_set_statuspktrdy_after_in
                (*pac::USB).ENDPOINT[0].TX_CSR |=
                    (pac::CSR0L_HOST_STATUS_PKT_MASK | pac::CSR0L_HOST_TX_PKT_RDY_MASK) as u16;
                in_ep.state = EndpointState::RxComplete(in_ep.ep.xfr_count as usize);
                if let Some(waker) = in_ep.waker.as_mut() {
                    waker.wake_by_ref();
                }
            } else {
                // Request next packet
                // MSS_USBH_CIF_cep_set_request_in_pkt
                (*pac::USB).ENDPOINT[0].TX_CSR |= pac::CSR0L_HOST_IN_PKT_REQ_MASK as u16;
            }
        } else {
            log::error!(
                "usbh_cep: unknown state = IN {:?}, OUT {:?}",
                in_ep.state,
                out_ep.state
            );
        }
    });
}

extern "C" fn usbh_tx_complete(ep_num: u8, status: u8) {
    log::trace!("usbh_tx_complete: ep={}, status={}", ep_num, status);
}

extern "C" fn usbh_rx(ep_num: u8, status: u8) {
    log::trace!("usbh_rx: ep={}, status={}", ep_num, status);
}

extern "C" fn usbh_dma_handler(
    ep_num: pac::mss_usb_ep_num_t,
    dma_dir: pac::mss_usb_dma_dir_t,
    status: u8,
    dma_addr_val: u32,
) {
    log::trace!(
        "usbh_dma_handler: ep={}, dir={:?}, status={}, addr=0x{:x}",
        ep_num,
        dma_dir,
        status,
        dma_addr_val
    );
}

extern "C" fn usbh_connect(
    target_speed: pac::mss_usb_device_speed_t,
    vbus_level: pac::mss_usb_vbus_level_t,
) {
    let speed = match target_speed {
        pac::mss_usb_device_speed_t_MSS_USB_DEVICE_FS => Speed::Full,
        pac::mss_usb_device_speed_t_MSS_USB_DEVICE_HS => Speed::High,
        _ => Speed::Low,
    };

    log::trace!("usbh_connect: speed={:?}, vbus={:?}", speed, vbus_level);

    #[allow(static_mut_refs)]
    critical_section::with(|_| unsafe {
        DEVICE_EVENT = Some(DeviceEvent::Connected(speed));
        if let Some(waker) = DEVICE_EVENT_WAKER.take() {
            waker.wake();
        }
    });
}

extern "C" fn usbh_disconnect() {
    log::trace!("usbh_disconnect");

    #[allow(static_mut_refs)]
    critical_section::with(|_| unsafe {
        DEVICE_EVENT = Some(DeviceEvent::Disconnected);
        if let Some(waker) = DEVICE_EVENT_WAKER.take() {
            waker.wake();
        }
    });
}
