use core::future::poll_fn;
use core::task::{Poll, Waker};
use embassy_usb_driver::{
    Direction, Endpoint, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo,
    EndpointType, Event, Speed,
};

use mpfs_hal::{pac, Peripheral};

use super::common::*;

// Endpoints are named relative to the host, so In is a device Tx and Out is a device Rx
// +1 for the control endpoint at index 0
static mut EP_IN_CONTROLLER: [Option<EndpointController>; NUM_ENDPOINTS + 1] =
    [const { None }; NUM_ENDPOINTS + 1];
static mut EP_OUT_CONTROLLER: [Option<EndpointController>; NUM_ENDPOINTS + 1] =
    [const { None }; NUM_ENDPOINTS + 1];

//------------------------------------------------------
// Driver

#[derive(Default)]
struct EndpointDetails {
    used: bool,
    fifo_addr: u16,
    fifo_size: u16,
}

pub struct UsbDriver<'a> {
    speed: Speed,
    phantom: core::marker::PhantomData<&'a ()>,
    // Endpoint number 1 maps to index 0
    in_endpoints_allocated: [EndpointDetails; NUM_ENDPOINTS],
    out_endpoints_allocated: [EndpointDetails; NUM_ENDPOINTS],
}

impl<'a> Default for UsbDriver<'a> {
    fn default() -> Self {
        UsbDriver {
            speed: Speed::High,
            phantom: core::marker::PhantomData,
            in_endpoints_allocated: [const {
                EndpointDetails {
                    used: false,
                    fifo_addr: 0,
                    fifo_size: 0,
                }
            }; NUM_ENDPOINTS],
            out_endpoints_allocated: [const {
                EndpointDetails {
                    used: false,
                    fifo_addr: 0,
                    fifo_size: 0,
                }
            }; NUM_ENDPOINTS],
        }
    }
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
    pub fn set_full_speed(&mut self) {
        self.speed = Speed::Full;
    }

    fn init(&self, control_packet_size: u16) {
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
            pac::MSS_USBD_CIF_init(speed_to_mss_value(self.speed));
            pac::MSS_USBD_CIF_cep_configure();
            EP_IN_CONTROLLER[0] = Some({
                let mut ep = EndpointController::default();
                ep.ep.max_pkt_size = control_packet_size;
                ep.state = EndpointState::Idle;
                ep
            });
            EP_OUT_CONTROLLER[0] = Some({
                let mut ep = EndpointController::default();
                ep.ep.max_pkt_size = control_packet_size;
                ep.state = EndpointState::Idle;
                ep
            });
            // MSS_USBD_CIF_dev_connect
            (*pac::USB).POWER |= pac::POWER_REG_SOFT_CONN_MASK as u8;
        }
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
                fifo_addr,
                fifo_size,
            };
            let ep = configure_endpoint_controller(
                Direction::Out,
                i + 1,
                endpoint_type,
                max_packet_size,
                fifo_addr,
                fifo_size,
                self.speed,
            );
            // TODO: We know nothing else will be using this endpoint yet (it hasn't been allocated), so we can safely do this without a lock
            unsafe {
                EP_OUT_CONTROLLER[i + 1] = Some(ep);
            }
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
            };
            let ep = configure_endpoint_controller(
                Direction::In,
                i + 1,
                endpoint_type,
                max_packet_size,
                fifo_addr,
                fifo_size,
                self.speed,
            );
            // TODO: We know nothing else will be using this endpoint yet (it hasn't been allocated), so we can safely do this without a lock
            unsafe {
                EP_IN_CONTROLLER[i + 1] = Some(ep);
            }
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
        self.init(control_packet_size);
        for (i, ep) in self.in_endpoints_allocated.iter().enumerate() {
            if ep.used {
                configure_endpoint(Direction::In, i + 1);
            } else {
                break;
            }
        }
        for (i, ep) in self.out_endpoints_allocated.iter().enumerate() {
            if ep.used {
                configure_endpoint(Direction::Out, i + 1);
            } else {
                break;
            }
        }
        log::debug!("USB initialized");
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
        let index = self.info.addr.index();
        log::trace!("USB EndpointOut::read {}", index);
        self.wait_enabled().await;
        let mut aligned_buffer = data.as_ptr();

        unsafe {
            if aligned_buffer.align_offset(4) != 0 {
                let e = EP_OUT_CONTROLLER[index].as_mut().unwrap();
                log::warn!("EndpointIn:{} data should be 32-bit aligned", index);
                aligned_buffer = e.buffer_addr()[..data.len()].as_ptr();
            }
        }

        unsafe {
            let e = EP_OUT_CONTROLLER[index].as_mut().unwrap();
            critical_section::with(|_| {
                e.state = EndpointState::Rx;
                e.ep.buf_addr = aligned_buffer as *mut u8;
                e.ep.xfr_length = data.len() as u32;
            });
            pac::MSS_USBD_CIF_rx_ep_read_prepare(&mut e.ep);
        }

        let read_size = poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                if let EndpointState::RxComplete(i) =
                    EP_OUT_CONTROLLER[index].as_ref().unwrap().state
                {
                    Poll::Ready(i)
                } else {
                    EP_OUT_CONTROLLER[index].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;

        if data.as_ptr() != aligned_buffer {
            unsafe {
                let e = EP_OUT_CONTROLLER[index].as_mut().unwrap();
                data.copy_from_slice(&e.buffer_addr()[..data.len()]);
            }
        }

        Ok(read_size)
    }
}

impl<'a> embassy_usb_driver::Endpoint for EndpointOut<'a> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        log::trace!(
            "USB EndpointOut::wait_enabled {} WAITING",
            self.info.addr.index()
        );
        let index = self.info.addr.index();
        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                if EP_OUT_CONTROLLER[index].as_ref().unwrap().state != EndpointState::Disabled {
                    Poll::Ready(())
                } else {
                    EP_OUT_CONTROLLER[index].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;
        log::trace!(
            "USB EndpointOut::wait_enabled {} OK",
            self.info.addr.index()
        );
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
        let index = self.info.addr.index();
        log::trace!("USB EndpointIn::write {} : {:?}", index, data);
        self.wait_enabled().await;
        let mut aligned_buffer = data;

        #[allow(static_mut_refs)]
        unsafe {
            // We are reading values that won't be changed, so we can do this without a lock

            if data.as_ptr().align_offset(4) != 0 {
                let e = EP_IN_CONTROLLER[index].as_mut().unwrap();
                log::warn!("EndpointIn:{} data should be 32-bit aligned", index);
                e.buffer_addr()[..data.len()].copy_from_slice(data);
                aligned_buffer = &e.buffer_addr()[..data.len()];
            }
            critical_section::with(|_| {
                EP_IN_CONTROLLER[index].as_mut().unwrap().state = EndpointState::Tx;
            });
            let e = EP_IN_CONTROLLER[index].as_mut().unwrap();
            pac::MSS_USB_CIF_ep_write_pkt(
                e.ep.num,
                aligned_buffer.as_ptr() as *mut u8,
                e.ep.dma_enable,
                e.ep.dma_channel,
                e.ep.xfr_type,
                aligned_buffer.len() as u32,
                aligned_buffer.len() as u32,
            );
        }

        poll_fn(move |cx| {
            critical_section::with(|_| unsafe {
                if EP_IN_CONTROLLER[index].as_mut().unwrap().state == EndpointState::TxComplete {
                    EP_IN_CONTROLLER[index].as_mut().unwrap().state = EndpointState::Idle;
                    Poll::Ready(())
                } else {
                    EP_IN_CONTROLLER[index].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;
        log::trace!("USB EndpointIn::write {} OK", index);
        Ok(())
    }
}

impl<'a> embassy_usb_driver::Endpoint for EndpointIn<'a> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        log::trace!(
            "USB EndpointIn::wait_enabled {} WAITING",
            self.info.addr.index()
        );
        let index = self.info.addr.index();
        poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                if EP_IN_CONTROLLER[index].as_ref().unwrap().state != EndpointState::Disabled {
                    Poll::Ready(())
                } else {
                    EP_IN_CONTROLLER[index].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;
        log::trace!("USB EndpointIn::wait_enabled {} OK", self.info.addr.index());
    }
}

//------------------------------------------------------
// Endpoint config

fn configure_endpoint(direction: Direction, endpoint: usize) {
    critical_section::with(|_| unsafe {
        if direction == Direction::Out {
            if let Some(ep) = EP_OUT_CONTROLLER[endpoint].as_mut() {
                // log::debug!("Configuring OUT endpoint {:?}", ep);
                pac::MSS_USBD_CIF_rx_ep_configure(&mut ep.ep);
                ep.state = EndpointState::Disabled;
            }
        } else {
            if let Some(ep) = EP_IN_CONTROLLER[endpoint].as_mut() {
                // log::debug!("Configuring IN endpoint {:?}", ep);
                pac::MSS_USBD_CIF_tx_ep_configure(&mut ep.ep);
                ep.state = EndpointState::Disabled;
            }
        }
    });
}

//------------------------------------------------------
// ControlPipe
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
                    if EP_OUT_CONTROLLER[0].as_mut().unwrap().state == EndpointState::Setup {
                        EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Idle;
                        Poll::Ready(())
                    } else {
                        EP_OUT_CONTROLLER[0].as_mut().unwrap().waker = Some(cx.waker().clone());
                        Poll::Pending
                    }
                })
            })
            .await;

            let mut setup_packet = [0; 8];
            read_control_packet(&mut setup_packet);
            // For some reason, sometimes the packet wasn't actually ready.
            // If we loop here and wait for the next interrupt, we seem to get the right packet.
            if setup_packet != [0; 8] {
                log::trace!("USB ControlPipe::setup packet: {:x?}", setup_packet);
                return setup_packet;
            } else {
                log::warn!("Got empty setup packet");
            }
        }
    }

    // Host -> Device
    // If a setup packet is received while this is waiting, this must return `EndpointError::Disabled`, but this will never be able to happen
    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        // It does not seem like we need to do anything special when last is true

        critical_section::with(|_| unsafe {
            EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Rx;
        });
        read_control_packet(buf);

        let read_size = poll_fn(move |cx| {
            critical_section::with(|_| unsafe {
                if let EndpointState::RxComplete(i) = EP_OUT_CONTROLLER[0].as_mut().unwrap().state {
                    EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Idle;
                    Poll::Ready(i)
                } else {
                    EP_OUT_CONTROLLER[0].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;
        Ok(read_size)
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

        #[allow(static_mut_refs)]
        unsafe {
            if data.as_ptr().align_offset(4) != 0 {
                log::warn!("Control endpoint data should be 32-bit aligned");
                let e = EP_IN_CONTROLLER[0].as_mut().unwrap();
                e.buffer_addr()[..data.len()].copy_from_slice(data);
                aligned_buffer = &e.buffer_addr()[..data.len()];
            }
            let ep = EP_IN_CONTROLLER[0].as_mut().unwrap();
            ep.ep.buf_addr = aligned_buffer.as_ptr() as *mut u8;
            ep.ep.txn_length = aligned_buffer.len() as u32;
            // When last is true, we send a zero length packet after the transfer
            // This means that xfr_count + txn_length >= xfr_length
            // Otherwise, xfr_count + txn_length < xfr_length
            // The specifics of these numbers are otherwise not important
            ep.ep.xfr_count = 0;
            ep.ep.xfr_length = if last {
                0
            } else {
                1 + aligned_buffer.len() as u32
            };
            ep.state = if last {
                EndpointState::TxLast
            } else {
                EndpointState::Tx
            };
            pac::MSS_USBD_CIF_cep_write_pkt(&mut ep.ep);
        }
        poll_fn(move |cx| {
            critical_section::with(|_| unsafe {
                let state = &mut EP_IN_CONTROLLER[0].as_mut().unwrap().state;
                if *state == EndpointState::TxComplete {
                    *state = EndpointState::Idle;
                    Poll::Ready(Ok(()))
                } else if *state == EndpointState::TxReadyForNext {
                    Poll::Ready(Ok(()))
                } else if *state == EndpointState::Setup {
                    log::warn!(
                        "Control endpoint setup packet received while waiting for tx complete"
                    );
                    Poll::Ready(Err(EndpointError::Disabled))
                } else {
                    EP_IN_CONTROLLER[0].as_mut().unwrap().waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await?;

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
        unsafe {
            (*pac::USB).INDEXED_CSR.DEVICE_EP0.CSR0 =
                (pac::CSR0L_DEV_SEND_STALL_MASK | pac::CSR0L_DEV_SERVICED_RX_PKT_RDY_MASK) as u16;
        }
        log::trace!("USB ControlPipe::reject");
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

fn read_control_packet(buf: &mut [u8]) {
    unsafe {
        let ep = EP_OUT_CONTROLLER[0].as_mut().unwrap();
        ep.ep.buf_addr = buf.as_mut_ptr() as *mut u8;
        ep.ep.txn_length = buf.len() as u32;
        ep.ep.xfr_length = buf.len() as u32;
        pac::MSS_USBD_CIF_cep_read_pkt(&mut ep.ep);
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
                critical_section::with(|_| {
                    EP_IN_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Idle;
                    EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Idle;
                });
                for ep in 1..=NUM_ENDPOINTS {
                    configure_endpoint(Direction::In, ep);
                }
                for ep in 1..=NUM_ENDPOINTS {
                    configure_endpoint(Direction::Out, ep);
                }
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
        let state = if enabled {
            EndpointState::Idle
        } else {
            EndpointState::Disabled
        };
        critical_section::with(|_| unsafe {
            if ep_addr.direction() == Direction::Out {
                EP_OUT_CONTROLLER[ep_addr.index()].as_mut().unwrap().state = state;
            } else {
                EP_IN_CONTROLLER[ep_addr.index()].as_mut().unwrap().state = state;
            }
        });
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        log::trace!(
            "USB Bus::endpoint_set_stalled: {:?}, {:?}",
            ep_addr,
            stalled
        );
        unsafe {
            if ep_addr.direction() == Direction::Out {
                let reg = &mut (*pac::USB).ENDPOINT[ep_addr.index()].RX_CSR;
                if stalled {
                    *reg |= (pac::RxCSRL_REG_EPN_SEND_STALL_MASK
                        | pac::RxCSRL_REG_EPN_RX_PKT_RDY_MASK) as u16;
                } else {
                    let mut reg_val = *reg;
                    reg_val &= !(pac::RxCSRL_REG_EPN_SEND_STALL_MASK as u16);
                    reg_val |= pac::RxCSRL_REG_EPN_RX_PKT_RDY_MASK as u16;
                    *reg = reg_val;
                }
            } else {
                let reg = &mut (*pac::USB).ENDPOINT[ep_addr.index()].TX_CSR;
                if stalled {
                    *reg |= pac::TxCSRL_REG_EPN_SEND_STALL_MASK as u16;
                } else {
                    *reg &= !(pac::TxCSRL_REG_EPN_SEND_STALL_MASK as u16);
                }
            }
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let val = unsafe {
            if ep_addr.direction() == Direction::Out {
                (*pac::USB).ENDPOINT[ep_addr.index()].RX_CSR
                    & pac::RxCSRL_REG_EPN_STALL_SENT_MASK as u16
                    != 0
            } else {
                (*pac::USB).ENDPOINT[ep_addr.index()].TX_CSR
                    & pac::TxCSRL_REG_EPN_STALL_SENT_MASK as u16
                    != 0
            }
        };
        log::trace!("USB Bus::endpoint_is_stalled: {:?}, {:?}", ep_addr, val);
        val
    }

    async fn enable(&mut self) {}
    async fn disable(&mut self) {}

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        // Maybe TODO: Does the MPFS support this? The platform code seems to hint at its existence,
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
    usbd_reset: Some(usbd_reset),
    usbd_suspend: Some(usbd_suspend),
    usbd_resume: Some(usbd_resume),
    usbd_disconnect: Some(usbd_disconnect),
    usbd_dma_handler: Some(usbd_dma_handler),

    // This is never called
    usbd_sof: None,
    // These will never be called, since we don't use the CIF's cep_state, which is what dictates
    // whether setup, rx, or tx_complete is called
    // Instead we look at the state of the endpoint controller directly in cep_setup
    usbd_cep_rx: None,
    usbd_cep_tx_complete: None,
};

extern "C" fn usbd_cep_setup(status: u8) {
    unsafe {
        log::trace!(
            "usbd_cep_setup: {:?} {:?} {:?}",
            status,
            EP_OUT_CONTROLLER[0].as_mut().unwrap().state,
            EP_IN_CONTROLLER[0].as_mut().unwrap().state
        );
    }
    // This is called in three cases:
    // 1. When the control endpoint is stalled
    // 2. When the control endpoint recieves a setup packet before the setup has completed
    // 3. When the control endpoint recieves a normal setup packet
    //
    // The CIF clears the PHY of the stall in the first case already, so there's nothing left to do
    // The other two we can handle the same (I think?)
    critical_section::with(|_| unsafe {
        let read_ready = pac::MSS_USB_CIF_cep_is_rxpktrdy() != 0;
        if EP_OUT_CONTROLLER[0].as_mut().unwrap().state == EndpointState::Rx && read_ready {
            // MSS_USB_CIF_cep_rx_byte_count
            let len = (*pac::USB).INDEXED_CSR.DEVICE_EP0.COUNT0 & pac::COUNT0_REG_MASK as u16;
            EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::RxComplete(len as usize);
        } else if EP_IN_CONTROLLER[0].as_mut().unwrap().state == EndpointState::Tx {
            EP_IN_CONTROLLER[0].as_mut().unwrap().state = EndpointState::TxReadyForNext;
        } else if EP_IN_CONTROLLER[0].as_mut().unwrap().state == EndpointState::TxLast {
            EP_IN_CONTROLLER[0].as_mut().unwrap().state = EndpointState::TxComplete;
        } else if read_ready {
            EP_OUT_CONTROLLER[0].as_mut().unwrap().state = EndpointState::Setup;
        }

        #[allow(static_mut_refs)]
        if let Some(waker) = EP_OUT_CONTROLLER[0].as_mut().unwrap().waker.as_mut() {
            waker.wake_by_ref();
        }
        if let Some(waker) = EP_IN_CONTROLLER[0].as_mut().unwrap().waker.as_mut() {
            waker.wake_by_ref();
        }
    });
}

unsafe fn rx_complete(num: usize, received_count: u32) {
    critical_section::with(|_| {
        EP_OUT_CONTROLLER[num as usize].as_mut().unwrap().state =
            EndpointState::RxComplete(received_count as usize);
    });
    if let Some(waker) = EP_OUT_CONTROLLER[num as usize]
        .as_mut()
        .unwrap()
        .waker
        .as_mut()
    {
        waker.wake_by_ref();
    }
}

extern "C" fn usbd_ep_rx(num: pac::mss_usb_ep_num_t, status: u8) {
    unsafe {
        let received_count = (*pac::USB).ENDPOINT[num as usize].RX_COUNT as u32;
        let ep = EP_OUT_CONTROLLER[num as usize].as_ref().unwrap();
        log::trace!(
            "usbd_ep_rx {:?}: {:?}; Received count {:?}",
            num,
            status,
            received_count
        );

        // DMA
        if ep.ep.dma_enable != 0 {
            if ((*pac::USB).ENDPOINT[num as usize].RX_CSR
                & pac::RxCSRL_REG_EPN_DMA_MODE_MASK as u16)
                == 0
            {
                log::trace!("DMA mode 0");

                // MSS_USB_CIF_dma_write_count
                (*pac::USB).DMA_CHANNEL[ep.ep.dma_channel as usize].COUNT = received_count;
                // MSS_USB_CIF_dma_start_xfr
                (*pac::USB).DMA_CHANNEL[ep.ep.dma_channel as usize].CNTL |=
                    pac::DMA_CNTL_REG_START_XFR_MASK;
                return; // A DMA interrupt will be triggered
            } else {
                log::trace!("DMA mode 1");
                // MSS_USB_CIF_dma_stop_xfr
                (*pac::USB).DMA_CHANNEL[ep.ep.dma_channel as usize].CNTL &=
                    !pac::DMA_CNTL_REG_START_XFR_MASK;
                pac::MSS_USB_CIF_rx_ep_clr_autoclr(num);
                // Rest is the same as for non-DMA mode
            }
        }

        if received_count > 0 {
            pac::MSS_USB_CIF_read_rx_fifo(
                num,
                EP_OUT_CONTROLLER[num as usize]
                    .as_mut()
                    .unwrap()
                    .ep
                    .buf_addr as *mut core::ffi::c_void,
                received_count,
            );
        }
        // MSS_USB_CIF_rx_ep_clr_rxpktrdy
        (*pac::USB).ENDPOINT[num as usize].RX_CSR &= !(pac::RxCSRL_REG_EPN_RX_PKT_RDY_MASK as u16);
        rx_complete(num as usize, received_count);
    }
}

fn tx_complete(ep_num: usize) {
    critical_section::with(|_| unsafe {
        EP_IN_CONTROLLER[ep_num].as_mut().unwrap().state = EndpointState::TxComplete;
        if let Some(waker) = EP_IN_CONTROLLER[ep_num].as_mut().unwrap().waker.as_mut() {
            waker.wake_by_ref();
        }
    });
}

extern "C" fn usbd_ep_tx_complete(num: pac::mss_usb_ep_num_t, status: u8) {
    log::trace!("usbd_ep_tx_complete {:?}: {:?}", num, status);
    // unsafe {
    //     let dma_channel = EP_IN_CONTROLLER[num as usize]
    //         .as_mut()
    //         .unwrap()
    //         .ep
    //         .dma_channel;
    //     if dma_channel != pac::mss_usb_dma_channel_t_MSS_USB_DMA_CHANNEL_NA {
    //         log::trace!(
    //             "DMA Tx {} complete on channel {:?}: {:?}",
    //             num,
    //             dma_channel,
    //             &(*pac::USB).DMA_CHANNEL[dma_channel as usize]
    //         );
    //     }
    // }
    tx_complete(num as usize);
}

extern "C" fn usbd_dma_handler(
    ep_num: pac::mss_usb_ep_num_t,
    dma_dir: pac::mss_usb_dma_dir_t,
    status: u8,
    dma_addr_val: u32,
) {
    log::trace!(
        "usbd_dma_handler {:?}: {:?}, {:?}, 0x{:x?}",
        ep_num,
        dma_dir,
        status,
        dma_addr_val
    );
    if status == pac::DMA_XFR_ERROR as u8 {
        log::error!("DMA transfer error");
    } else {
        if dma_dir == pac::mss_usb_dma_dir_t_MSS_USB_DMA_READ {
            // Tx
            unsafe {
                // This triggers a TX complete interrupt
                // MSS_USB_CIF_tx_ep_set_txpktrdy
                (*pac::USB).ENDPOINT[ep_num as usize].TX_CSR |=
                    pac::TxCSRL_REG_EPN_TX_PKT_RDY_MASK as u16;
            }
        } else {
            // Rx
            log::trace!("DMA RX {:?}", ep_num);
            unsafe {
                let received_count = dma_addr_val
                    - EP_OUT_CONTROLLER[ep_num as usize]
                        .as_ref()
                        .unwrap()
                        .ep
                        .buf_addr as u32;
                rx_complete(ep_num as usize, received_count);
            }
        }
    }
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
