use aligned::{Aligned, A4};
use alloc::rc::Rc;
use core::cell::RefCell;
use core::future::poll_fn;
use core::task::{Poll, Waker};
use embassy_futures::select;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb_driver::host::{
    channel, ChannelError, DeviceEvent, HostError, SetupPacket, TimeoutConfig, UsbChannel,
    UsbHostDriver,
};
use embassy_usb_driver::{Direction, EndpointInfo, EndpointType, Speed};

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
pub struct UsbHost {
    in_channels_allocated: Rc<RefCell<[EndpointDetails; NUM_ENDPOINTS]>>,
    out_channels_allocated: Rc<RefCell<[EndpointDetails; NUM_ENDPOINTS]>>,
    fs_only: bool,
}

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
    /// Must be called before `start`
    // TODO: Does this work?
    pub fn disable_high_speed(&mut self) {
        self.fs_only = true;
    }

    /// Initialize the USB host
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

            if !self.fs_only {
                (*pac::USB).POWER |= pac::POWER_REG_ENABLE_HS_MASK as u8;
            }

            pac::MSS_USBH_CIF_init();
        }
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
                        // Wait first, since we already set this when we first called `start`
                        Timer::after(Duration::from_millis(5000)).await;
                        // Set session bit periodically to check to see if device is connected
                        (*pac::USB).DEV_CTRL |= pac::DEV_CTRL_SESSION_MASK as u8;
                    };

                    let event_future = poll_fn(|cx| {
                        critical_section::with(|_| {
                            if let Some(DeviceEvent::Connected(_)) = DEVICE_EVENT {
                                let event = DEVICE_EVENT.take().unwrap();
                                Poll::Ready(event)
                            } else {
                                DEVICE_EVENT_WAKER = Some(cx.waker().clone());
                                Poll::Pending
                            }
                        })
                    });

                    let event = select::select(session_future, event_future).await;
                    if event.is_second() {
                        log::debug!("USB connected");
                        Timer::after(Duration::from_millis(100)).await;
                        self.bus_reset().await;
                        let event = DeviceEvent::Connected(
                            if !self.fs_only
                                && (*pac::USB).POWER & pac::POWER_REG_HS_MODE_MASK as u8 != 0
                            {
                                Speed::High
                            } else {
                                Speed::Full
                            },
                        );
                        critical_section::with(|_| {
                            CONNECTED = event;
                        });
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
                    log::debug!("USB disconnected");
                    return embassy_usb_driver::host::DeviceEvent::Disconnected;
                }
            }
        }
    }

    async fn bus_reset(&self) {
        log::trace!("USBH bus_reset");
        unsafe {
            // MSS_USBH_CIF_assert_bus_reset
            (*pac::USB).POWER |= pac::POWER_REG_BUS_RESET_SIGNAL_MASK as u8;
            Timer::after(Duration::from_millis(50)).await;
            // MSS_USBH_CIF_clr_bus_reset
            (*pac::USB).POWER &= !pac::POWER_REG_BUS_RESET_SIGNAL_MASK as u8;
            // Without this wait, the device becomes unhappy
            Timer::after(Duration::from_millis(40)).await;
        }
    }

    fn alloc_channel<T: channel::Type, D: channel::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        pre: bool,
    ) -> Result<Self::Channel<T, D>, HostError> {
        assert!(!pre, "Low speed devices aren't supported");
        log::trace!(
            "USBH alloc_channel {} {:?}; Control: {:?}; In: {:?}; Out: {:?}",
            addr,
            endpoint,
            T::ep_type() == EndpointType::Control,
            D::is_in(),
            D::is_out()
        );
        let index = endpoint.addr.index();

        let (ep_in, ep_out) = if T::ep_type() == EndpointType::Control {
            let mut ep = default_ep();
            ep.num = 0;
            ep.xfr_type = pac::mss_usb_xfr_type_t_MSS_USB_XFR_CONTROL;
            ep.fifo_size = 64;
            ep.max_pkt_size = endpoint.max_packet_size;
            ep.interval = 32768; // Max value
            (
                Some(EndpointController {
                    ep,
                    ..EndpointController::default()
                }),
                Some(EndpointController {
                    ep,
                    ..EndpointController::default()
                }),
            )
        } else {
            let ep_in = if D::is_in() {
                unsafe {
                    if EP_IN_CONTROLLER[index].is_some() {
                        return Err(HostError::OutOfChannels);
                    }
                }
                let i = alloc_fifo_addr(
                    endpoint.max_packet_size,
                    &mut *self.in_channels_allocated.borrow_mut(),
                )?;

                Some(configure_endpoint_controller(
                    Direction::In,
                    index,
                    endpoint.ep_type,
                    endpoint.max_packet_size,
                    self.in_channels_allocated.borrow()[i],
                    speed(),
                ))
            } else {
                None
            };

            let ep_out = if D::is_out() {
                unsafe {
                    if EP_OUT_CONTROLLER[index].is_some() {
                        return Err(HostError::OutOfChannels);
                    }
                }
                let i = alloc_fifo_addr(
                    endpoint.max_packet_size,
                    &mut *self.out_channels_allocated.borrow_mut(),
                )?;

                Some(configure_endpoint_controller(
                    Direction::Out,
                    index,
                    endpoint.ep_type,
                    endpoint.max_packet_size,
                    self.out_channels_allocated.borrow()[i],
                    speed(),
                ))
            } else {
                None
            };
            (ep_in, ep_out)
        };
        unsafe {
            // We know nothing else will be using this endpoint yet (it hasn't been allocated), so we can safely do this without a lock
            if D::is_in() {
                EP_IN_CONTROLLER[index] = ep_in;
            }
            if D::is_out() {
                EP_OUT_CONTROLLER[index] = ep_out;
            }
        }
        let mut channel = Channel {
            index,
            phantom: core::marker::PhantomData,
        };
        channel.retarget_channel(addr, endpoint, pre)?;
        Ok(channel)
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
        log::trace!(
            "USBH control_in: setup={:?}; setup_bytes={:x?}",
            setup,
            setup.as_bytes()
        );

        let mut aligned_buffer = buf.as_ptr();
        unsafe {
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
        }
        let mut setup_buffer = Aligned::<A4, _>([0; 8]);
        setup_buffer
            .as_mut_slice()
            .copy_from_slice(setup.as_bytes());
        let read_size = loop {
            if let Ok(res) = with_timeout(Duration::from_millis(1000), async {
                unsafe {
                    pac::MSS_USBH_CIF_load_tx_fifo(
                        0,
                        setup_buffer.as_ptr() as *mut core::ffi::c_void,
                        8,
                    );
                    // MSS_USBH_CIF_cep_set_setuppktrdy
                    (*pac::USB).ENDPOINT[0].TX_CSR |=
                        (pac::CSR0L_HOST_SETUP_PKT_MASK | pac::CSR0L_HOST_TX_PKT_RDY_MASK) as u16;
                }

                poll_fn(|cx| {
                    critical_section::with(|_| unsafe {
                        if let EndpointState::RxComplete(i) =
                            EP_IN_CONTROLLER[0].as_ref().unwrap().state
                        {
                            EP_IN_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Idle;
                            EP_IN_CONTROLLER[0].as_mut().unwrap().waker = None;
                            Poll::Ready(i)
                        } else {
                            EP_IN_CONTROLLER[0].as_mut().unwrap().waker = Some(cx.waker().clone());
                            Poll::Pending
                        }
                    })
                })
                .await
            })
            .await
            {
                break res;
            } else {
                log::warn!("control_in: timeout, retrying");
            }
        }?;

        if buf.as_ptr() != aligned_buffer {
            unsafe {
                let e = EP_IN_CONTROLLER[self.index].as_mut().unwrap();
                buf.copy_from_slice(&e.buffer_addr()[..buf.len()]);
            }
        }

        if read_size > 10 {
            log::trace!("control_in: read={:x?}... length={}", &buf[..10], read_size);
        } else {
            log::trace!("control_in: read={:x?}", &buf[..read_size]);
        }
        Ok(read_size)
    }

    async fn control_out(&mut self, setup: &SetupPacket, buf: &[u8]) -> Result<usize, ChannelError>
    where
        T: channel::IsControl,
        D: channel::IsOut,
    {
        log::trace!("control_out: setup={:?}, buf={:x?}", setup, buf);
        unsafe {
            let mut aligned_buffer = buf.as_ptr();
            if aligned_buffer.align_offset(4) != 0 && buf.len() > 0 {
                let e = EP_OUT_CONTROLLER[0].as_mut().unwrap();
                log::warn!("control_out: data should be 32-bit aligned");
                aligned_buffer = e.buffer_addr()[..buf.len()].as_ptr();
            }
            let ep = EP_OUT_CONTROLLER[0].as_mut().unwrap();
            ep.ep.buf_addr = aligned_buffer as *mut u8;
            ep.ep.xfr_length = buf.len() as u32;
            ep.ep.xfr_count = 0;
            ep.ep.txn_count = 0;
            ep.ep.xfr_type = pac::mss_usb_xfr_type_t_MSS_USB_XFR_CONTROL;

            critical_section::with(|_| {
                EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Setup;
            });
            let mut setup_buffer = Aligned::<A4, _>([0; 8]);
            setup_buffer
                .as_mut_slice()
                .copy_from_slice(setup.as_bytes());

            pac::MSS_USBH_CIF_load_tx_fifo(0, setup_buffer.as_ptr() as *mut core::ffi::c_void, 8);
            // MSS_USBH_CIF_cep_set_setuppktrdy
            (*pac::USB).ENDPOINT[0].TX_CSR |=
                (pac::CSR0L_HOST_SETUP_PKT_MASK | pac::CSR0L_HOST_TX_PKT_RDY_MASK) as u16;
        }

        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                if let EndpointState::TxComplete = EP_OUT_CONTROLLER[0].as_ref().unwrap().state {
                    EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Idle;
                    Poll::Ready(())
                } else {
                    EP_OUT_CONTROLLER[0].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;

        Ok(buf.len())
    }

    fn retarget_channel(
        &mut self,
        addr: u8,
        endpoint: &EndpointInfo,
        pre: bool,
    ) -> Result<(), HostError> {
        assert!(!pre, "Low speed devices aren't supported");
        log::trace!("USBH retarget_channel {} {:?}", addr, endpoint);
        unsafe {
            if T::ep_type() == EndpointType::Control {
                let ep = EP_IN_CONTROLLER[0].as_mut().unwrap();
                pac::MSS_USBH_CIF_cep_configure(&mut ep.ep);

                // Wait before setting speed, or else the device gets unhappy
                for _ in 0..200000 {
                    core::hint::spin_loop();
                }
                // pac::MSS_USBH_CIF_cep_set_type0_reg
                (*pac::USB).ENDPOINT[0].TX_TYPE =
                    (mss_speed() << pac::TYPE0_HOST_MP_TARGET_SPEED_SHIFT) as u8;
                // pac::MSS_USBH_CIF_tx_ep_set_target_func_addr(0, endpoint.addr);
                (*pac::USB).TAR[0].TX_FUNC_ADDR = (addr & 0x7F) as u8;
            } else {
                let index = endpoint.addr.index();
                if D::is_in() {
                    pac::MSS_USBH_CIF_rx_ep_mp_configure(
                        index as u8,
                        index as u8,
                        addr,
                        // Needed for hub?
                        0,
                        0,
                        0,
                        mss_speed(),
                        mss_interval(speed(), endpoint.ep_type),
                        mss_transfer_type(endpoint.ep_type),
                    );

                    let ep = EP_IN_CONTROLLER[index].as_mut().unwrap();
                    pac::MSS_USBH_CIF_rx_ep_configure(&mut ep.ep);
                }
                if D::is_out() {
                    pac::MSS_USBH_CIF_tx_ep_mp_configure(
                        index as u8,
                        index as u8,
                        addr,
                        // Needed for hub?
                        0,
                        0,
                        0,
                        mss_speed(),
                        mss_interval(speed(), endpoint.ep_type),
                        mss_transfer_type(endpoint.ep_type),
                    );

                    let ep = EP_OUT_CONTROLLER[index].as_mut().unwrap();
                    pac::MSS_USBH_CIF_tx_ep_configure(&mut ep.ep);
                }
            }
        }
        Ok(())
    }

    /// Configure the timeouts of this channel
    async fn set_timeout(&mut self, _timeout: TimeoutConfig) {
        log::warn!("set_timeout: not implemented");
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsIn,
    {
        log::trace!("USBH request_in");

        // based off MSS_USBH_read_in_pipe

        let mut aligned_buffer = buf.as_ptr();
        unsafe {
            let ep = EP_IN_CONTROLLER[self.index].as_mut().unwrap();
            if aligned_buffer.align_offset(4) != 0 {
                log::warn!("request_in: data should be 32-bit aligned");
                aligned_buffer = ep.buffer_addr()[..buf.len()].as_ptr();
            }
            critical_section::with(|_| {
                ep.state = EndpointState::Rx;
                if buf.len() > ep.ep.max_pkt_size as usize {
                    ep.ep.txn_length = ep.ep.max_pkt_size as u32;
                } else {
                    ep.ep.txn_length = buf.len() as u32;
                }
                ep.ep.buf_addr = aligned_buffer as *mut u8;
                ep.ep.xfr_length = buf.len() as u32;
                ep.ep.xfr_count = 0;
                ep.ep.txn_count = 0;
            });

            if ep.ep.dma_enable != 0 && T::ep_type() == EndpointType::Bulk {
                // MSS_USBH_CIF_rx_ep_set_reqpkt_count
                (*pac::USB).RQ_PKT_CNT[self.index] = ep.ep.xfr_length / ep.ep.max_pkt_size as u32;
                // MSS_USBH_CIF_rx_ep_set_autoreq;
                (*pac::USB).ENDPOINT[self.index].RX_CSR |=
                    pac::RXCSRH_HOST_EPN_ENABLE_AUTOREQ_MASK as u16;
            }

            pac::MSS_USB_CIF_rx_ep_read_prepare(
                ep.ep.num,
                ep.ep.buf_addr,
                ep.ep.dma_enable,
                ep.ep.dma_channel,
                ep.ep.xfr_type,
                ep.ep.xfr_length,
            );
            // MSS_USBH_CIF_rx_ep_set_reqpkt((mss_usb_ep_num_t)inpipe_num)
            (*pac::USB).ENDPOINT[self.index].RX_CSR |= pac::RXCSRL_HOST_EPN_IN_PKT_REQ_MASK as u16;
        }

        let read = poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                let ep = EP_IN_CONTROLLER[self.index].as_mut().unwrap();
                if let EndpointState::RxComplete(i) = ep.state {
                    ep.state = EndpointState::Idle;
                    Poll::Ready(i)
                } else {
                    ep.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await?;

        if buf.as_ptr() != aligned_buffer {
            unsafe {
                let e = EP_IN_CONTROLLER[self.index].as_mut().unwrap();
                buf.copy_from_slice(&e.buffer_addr()[..buf.len()]);
            }
        }

        log::trace!("USBH request_in: read={:x?}", &buf[..read]);

        Ok(read)
    }

    async fn request_out(&mut self, buf: &[u8]) -> Result<usize, ChannelError>
    where
        D: channel::IsOut,
    {
        // Based off of MSS_USBH_write_out_pipe
        log::trace!("USBH request_out");

        unsafe {
            let mut aligned_buffer = buf.as_ptr();
            let ep = EP_OUT_CONTROLLER[self.index].as_mut().unwrap();
            if aligned_buffer.align_offset(4) != 0 {
                log::warn!("request_out: data should be 32-bit aligned");
                aligned_buffer = ep.buffer_addr()[..buf.len()].as_ptr();
            }
            critical_section::with(|_| {
                ep.state = EndpointState::Tx;
                if buf.len() > ep.ep.max_pkt_size as usize {
                    ep.ep.txn_length = ep.ep.max_pkt_size as u32;
                } else {
                    ep.ep.txn_length = buf.len() as u32;
                }
                ep.ep.buf_addr = aligned_buffer as *mut u8;
                ep.ep.xfr_length = buf.len() as u32;
                ep.ep.xfr_count = 0;
                ep.ep.txn_count = 0;
            });

            pac::MSS_USB_CIF_ep_write_pkt(
                ep.ep.num,
                ep.ep.buf_addr,
                ep.ep.dma_enable,
                ep.ep.dma_channel,
                ep.ep.xfr_type,
                ep.ep.xfr_length,
                ep.ep.txn_length,
            );
        }

        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                let ep = EP_OUT_CONTROLLER[self.index].as_mut().unwrap();
                if let EndpointState::TxComplete = ep.state {
                    ep.state = EndpointState::Idle;
                    Poll::Ready(())
                } else {
                    ep.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;

        Ok(buf.len())
    }
}

fn speed() -> Speed {
    unsafe {
        match CONNECTED {
            DeviceEvent::Connected(speed) => speed,
            DeviceEvent::Disconnected => Speed::Low,
        }
    }
}

fn mss_speed() -> pac::mss_usb_device_speed_t {
    speed_to_mss_value(speed())
}

pub fn mss_interval(_speed: Speed, endpoint_type: EndpointType) -> u32 {
    match endpoint_type {
        EndpointType::Interrupt => 1,   // Every frame/microframe
        EndpointType::Isochronous => 1, // Every frame/microframe
        EndpointType::Bulk => {
            32768 // Max NAK limit value
        }
        EndpointType::Control => 0, // Control endpoints don't use interval
    }
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

fn ep_state_to_string(state: u8) -> &'static str {
    match state as u32 {
        pac::mss_usb_ep_state_t_MSS_USB_EP_TXN_SUCCESS => "Success",
        pac::mss_usb_ep_state_t_MSS_USB_EP_NAK_TOUT => "NAK Timeout",
        pac::mss_usb_ep_state_t_MSS_USB_EP_NO_RESPONSE => "No Response",
        pac::mss_usb_ep_state_t_MSS_USB_EP_STALL_RCVD => "Stall Received",
        pac::mss_usb_ep_state_t_MSS_USB_EP_XFR_SUCCESS => "Transfer Success",
        pac::mss_usb_ep_state_t_MSS_USB_EP_ABORTED => "Aborted",
        _ => "Unknown",
    }
}

extern "C" fn usbh_cep(status: u8) {
    log::trace!("usbh_cep: Entering with status={}", status);

    unsafe {
        let in_ep = &mut EP_IN_CONTROLLER[0].as_mut().unwrap();
        let out_ep = &mut EP_OUT_CONTROLLER[0].as_mut().unwrap();
        // log::debug!("IN: {:?}, OUT: {:?}", in_ep.state, out_ep.state);

        critical_section::with(|_| {
            if status != pac::mss_usb_ep_state_t_MSS_USB_EP_TXN_SUCCESS as u8 {
                log::error!(
                    "usbh_cep: non-success status: {}",
                    ep_state_to_string(status)
                );
                if in_ep.state == EndpointState::Setup || in_ep.state == EndpointState::Rx {
                    in_ep.state = EndpointState::RxComplete(Err(ChannelError::BadResponse));
                    return;
                }
            }
            if in_ep.state == EndpointState::Setup {
                // log::debug!("usbh_cep: Processing IN Setup -> Rx transition");
                in_ep.state = EndpointState::Rx;
                (*pac::USB).ENDPOINT[0].TX_CSR |= pac::CSR0L_HOST_IN_PKT_REQ_MASK as u16;
                // log::debug!("usbh_cep: Requested first IN packet");
            } else if in_ep.state == EndpointState::Rx {
                // log::debug!("usbh_cep: Processing IN Rx state");
                pac::MSS_USBH_CIF_cep_read_pkt(&mut in_ep.ep);
                // log::debug!(
                //     "usbh_cep: Read packet - count={}, length={}",
                //     in_ep.ep.xfr_count,
                //     in_ep.ep.xfr_length
                // );

                if in_ep.ep.xfr_count == in_ep.ep.xfr_length {
                    // log::debug!("usbh_cep: Transfer complete, sending status packet");
                    (*pac::USB).ENDPOINT[0].TX_CSR |=
                        (pac::CSR0L_HOST_STATUS_PKT_MASK | pac::CSR0L_HOST_TX_PKT_RDY_MASK) as u16;
                    in_ep.state = EndpointState::RxComplete(Ok(in_ep.ep.xfr_count as usize));
                    if let Some(waker) = in_ep.waker.as_mut() {
                        // log::debug!("usbh_cep: Waking up waiting task for IN");
                        waker.wake_by_ref();
                    }
                } else {
                    // log::debug!("usbh_cep: Requesting next packet");
                    (*pac::USB).ENDPOINT[0].TX_CSR |= pac::CSR0L_HOST_IN_PKT_REQ_MASK as u16;
                }
            } else if out_ep.state == EndpointState::Setup {
                // log::debug!("usbh_cep: Processing OUT Setup state");
                if out_ep.ep.xfr_length == 0 {
                    // log::debug!("usbh_cep: Zero-length OUT packet, sending status packet");
                    (*pac::USB).ENDPOINT[0].TX_CSR |=
                        (pac::CSR0L_HOST_STATUS_PKT_MASK | pac::CSR0L_HOST_IN_PKT_REQ_MASK) as u16;
                    out_ep.state = EndpointState::TxLast;
                } else {
                    todo!("usbh_cep: setup OUT with non-zero length");
                }
            } else if out_ep.state == EndpointState::TxLast {
                // log::debug!("usbh_cep: Processing OUT TxLast state");
                pac::MSS_USBH_CIF_cep_clr_statusRxpktrdy();
                out_ep.state = EndpointState::TxComplete;
                if let Some(waker) = out_ep.waker.as_mut() {
                    // log::debug!("usbh_cep: Waking up waiting task");
                    waker.wake_by_ref();
                }
            } else {
                log::warn!(
                    "usbh_cep: Unexpected state combination - IN {:?}, OUT {:?}",
                    in_ep.state,
                    out_ep.state
                );
            }
        });
    }
}

extern "C" fn usbh_tx_complete(ep_num: u8, status: u8) {
    log::trace!("usbh_tx_complete: ep={}, status={}", ep_num, status);

    critical_section::with(|_| unsafe {
        let ep = EP_OUT_CONTROLLER[ep_num as usize].as_mut().unwrap();
        ep.state = EndpointState::TxComplete;
        if let Some(waker) = ep.waker.as_mut() {
            waker.wake_by_ref();
        }
    })
}

extern "C" fn usbh_rx(ep_num: u8, status: u8) {
    log::trace!("usbh_rx: ep={}, status={}", ep_num, status);
    if status != 0 {
        log::error!(
            "usbh_rx: non-success status: {}",
            ep_state_to_string(status)
        );
        critical_section::with(|_| unsafe {
            let ep = EP_IN_CONTROLLER[ep_num as usize].as_mut().unwrap();
            let err = match status as u32 {
                pac::RX_EP_STALL_ERROR => ChannelError::Stall,
                _ => ChannelError::BadResponse,
            };
            ep.state = EndpointState::RxComplete(Err(err));
            if let Some(waker) = ep.waker.as_mut() {
                waker.wake_by_ref();
            }
        });
    }

    let index = ep_num as usize;
    unsafe {
        // MSS_USB_CIF_rx_ep_is_rxpktrdy
        if (*pac::USB).ENDPOINT[index].RX_CSR & pac::RxCSRL_REG_EPN_RX_PKT_RDY_MASK as u16 != 0 {
            let received_count = (*pac::USB).ENDPOINT[index].RX_COUNT as u32;
            let ep = EP_IN_CONTROLLER[index].as_mut().unwrap();

            if received_count > 0 {
                pac::MSS_USB_CIF_read_rx_fifo(
                    ep_num as u32,
                    ep.ep.buf_addr as *mut core::ffi::c_void,
                    received_count,
                );
                ep.ep.xfr_count += received_count;

                // MSS_USB_CIF_rx_ep_clr_rxpktrdy
                (*pac::USB).ENDPOINT[index].RX_CSR &= !(pac::RxCSRL_REG_EPN_RX_PKT_RDY_MASK as u16);
            }
            if ep.ep.xfr_count >= ep.ep.xfr_length {
                // Transfer complete
                ep.state = EndpointState::RxComplete(Ok(ep.ep.xfr_count as usize));
                if let Some(waker) = ep.waker.as_mut() {
                    waker.wake_by_ref();
                }
            } else {
                // Request next packet
                // MSS_USBH_CIF_rx_ep_set_reqpkt((mss_usb_ep_num_t)inpipe_num)
                (*pac::USB).ENDPOINT[index].RX_CSR |= pac::RXCSRL_HOST_EPN_IN_PKT_REQ_MASK as u16;
            }
        }
    }
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
    if dma_dir == pac::mss_usb_dma_dir_t_MSS_USB_DMA_READ {
        // For some reason, this is the TX direction
        unsafe {
            // TODO: Handle ZLPs properly

            // This triggers a TX packet ready interrupt
            (*pac::USB).ENDPOINT[ep_num as usize].TX_CSR |=
                pac::TxCSRL_REG_EPN_TX_PKT_RDY_MASK as u16;
        }
    } else {
        // TODO Bulk Rx
    }
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
