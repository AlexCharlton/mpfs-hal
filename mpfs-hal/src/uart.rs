use crate::pac;
use core::task::Poll;
use embassy_embedded_hal::SetConfig;
use paste::paste;

pub trait UartPeripheral: crate::Peripheral {
    #[doc(hidden)]
    fn address(&self) -> *mut pac::mss_uart_instance_t;
    #[doc(hidden)]
    fn number(&self) -> u8;
}

pub trait UartRxPeripheral: crate::Peripheral + UartPeripheral {}

pub trait UartTxPeripheral: crate::Peripheral + UartPeripheral {}

//-------------------------------------------------------------------
// Create the UART peripherals

macro_rules! impl_uart {
    ($n:expr) => {
        paste! {
            impl_uart!([<Uart $n>], [<UartRx $n>], [<UartTx $n>], [<UART_RX_TAKEN $n>], [<UART_TX_TAKEN $n>], $n, [<g_mss_uart $n _lo>]);
        }
    };

    // E.g. impl_uart!(UART0, UART0_RX, UART0_TX, UART0_RX_TAKEN, UART0_TX_TAKEN, 0, g_mss_uart_0_lo);
    ($UART:ident, $UART_RX:ident, $UART_TX:ident, $UART_RX_TAKEN:ident, $UART_TX_TAKEN:ident, $num:expr, $instance:ident) => {
        pub struct $UART {
            _private: (),
        }

        pub struct $UART_RX {
            _private: (),
        }

        pub struct $UART_TX {
            _private: (),
        }

        static mut $UART_RX_TAKEN: bool = false;
        static mut $UART_TX_TAKEN: bool = false;

        impl crate::Peripheral for $UART {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $UART_RX_TAKEN || $UART_TX_TAKEN {
                        None
                    } else {
                        $UART_RX_TAKEN = true;
                        $UART_TX_TAKEN = true;
                        Some(Self { _private: () })
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }

        impl crate::Peripheral for $UART_RX {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $UART_RX_TAKEN {
                        None
                    } else {
                        $UART_RX_TAKEN = true;
                        Some(Self { _private: () })
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }

        impl crate::Peripheral for $UART_TX {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $UART_TX_TAKEN {
                        None
                    } else {
                        $UART_TX_TAKEN = true;
                        Some(Self { _private: () })
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }

        impl UartPeripheral for $UART {
            fn address(&self) -> *mut pac::mss_uart_instance_t {
                &raw mut pac::$instance
            }

            fn number(&self) -> u8 {
                $num
            }
        }

        impl UartPeripheral for $UART_RX {
            fn address(&self) -> *mut pac::mss_uart_instance_t {
                &raw mut pac::$instance
            }

            fn number(&self) -> u8 {
                $num
            }
        }

        impl UartRxPeripheral for $UART_RX {}

        impl UartPeripheral for $UART_TX {
            fn address(&self) -> *mut pac::mss_uart_instance_t {
                &raw mut pac::$instance
            }

            fn number(&self) -> u8 {
                $num
            }
        }

        impl UartTxPeripheral for $UART_TX {}
    };
}

macro_rules! impl_uarts {
    ($($n:expr),*) => {
        $(impl_uart!($n);)*
    };
}

// Generates UART0, UART1, UART2, UART3, UART4
impl_uarts!(0, 1, 2, 3, 4);

//-------------------------------------------------------------------
// Configs

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UartConfig {
    pub baud_rate: BaudRate,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud_rate: BaudRate::Baud115200,
            data_bits: DataBits::Data8,
            stop_bits: StopBits::One,
            parity: Parity::NoParity,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BaudRate {
    Baud110,
    Baud300,
    Baud600,
    Baud1200,
    Baud2400,
    Baud4800,
    Baud9600,
    Baud19200,
    Baud38400,
    Baud57600,
    Baud115200,
    Baud230400,
    Baud460800,
    Baud921600,
}

impl BaudRate {
    pub fn value(&self) -> u32 {
        match self {
            BaudRate::Baud110 => pac::MSS_UART_110_BAUD,
            BaudRate::Baud300 => pac::MSS_UART_300_BAUD,
            BaudRate::Baud600 => pac::MSS_UART_600_BAUD,
            BaudRate::Baud1200 => pac::MSS_UART_1200_BAUD,
            BaudRate::Baud2400 => pac::MSS_UART_2400_BAUD,
            BaudRate::Baud4800 => pac::MSS_UART_4800_BAUD,
            BaudRate::Baud9600 => pac::MSS_UART_9600_BAUD,
            BaudRate::Baud19200 => pac::MSS_UART_19200_BAUD,
            BaudRate::Baud38400 => pac::MSS_UART_38400_BAUD,
            BaudRate::Baud57600 => pac::MSS_UART_57600_BAUD,
            BaudRate::Baud115200 => pac::MSS_UART_115200_BAUD,
            BaudRate::Baud230400 => pac::MSS_UART_230400_BAUD,
            BaudRate::Baud460800 => pac::MSS_UART_460800_BAUD,
            BaudRate::Baud921600 => pac::MSS_UART_921600_BAUD,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DataBits {
    Data5,
    Data6,
    Data7,
    Data8,
}

impl DataBits {
    pub fn value(&self) -> u8 {
        match self {
            DataBits::Data5 => pac::MSS_UART_DATA_5_BITS,
            DataBits::Data6 => pac::MSS_UART_DATA_6_BITS,
            DataBits::Data7 => pac::MSS_UART_DATA_7_BITS,
            DataBits::Data8 => pac::MSS_UART_DATA_8_BITS,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StopBits {
    One,
    OneHalf,
    Two,
}

impl StopBits {
    pub fn value(&self) -> u8 {
        match self {
            StopBits::One => pac::MSS_UART_ONE_STOP_BIT,
            StopBits::OneHalf => pac::MSS_UART_ONEHALF_STOP_BIT,
            StopBits::Two => pac::MSS_UART_TWO_STOP_BITS,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Parity {
    NoParity,
    OddParity,
    EvenParity,
    StickParity0,
    StickParity1,
}

impl Parity {
    pub fn value(&self) -> u8 {
        match self {
            Parity::NoParity => pac::MSS_UART_NO_PARITY,
            Parity::OddParity => pac::MSS_UART_ODD_PARITY,
            Parity::EvenParity => pac::MSS_UART_EVEN_PARITY,
            Parity::StickParity0 => pac::MSS_UART_STICK_PARITY_0,
            Parity::StickParity1 => pac::MSS_UART_STICK_PARITY_1,
        }
    }
}

//-------------------------------------------------------------------

pub(crate) fn init_uart() {
    unsafe {
        // It just so happens that the UARTs are the first peripherals on the MPFS
        for i in 0..NUM_UARTS {
            pac::mss_config_clk_rst(
                i as u32,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
            );
        }
    }
}

unsafe fn init_uart_interrupt(num: usize) {
    // It just so happens that the UARTs are the first peripherals on the MPFS
    pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_MMUART0_INT_OFFSET + num as u32, 2);

    let uart = match num {
        0 => &raw mut pac::g_mss_uart0_lo,
        1 => &raw mut pac::g_mss_uart1_lo,
        2 => &raw mut pac::g_mss_uart2_lo,
        3 => &raw mut pac::g_mss_uart3_lo,
        4 => &raw mut pac::g_mss_uart4_lo,
        _ => panic!("Invalid UART number"),
    };
    pac::MSS_UART_set_rx_handler(
        uart,
        Some(uart_rx_handler),
        pac::mss_uart_rx_trig_level_t_MSS_UART_FIFO_SINGLE_BYTE,
    );
    pac::MSS_UART_enable_irq(uart, (pac::MSS_UART_RBF_IRQ | pac::MSS_UART_TBE_IRQ) as u16);
}

//-------------------------------------------------------------------
pub struct Uart<T: UartPeripheral> {
    rx: UartRx<T>,
    tx: UartTx<T>,
}

impl<T: UartPeripheral> Uart<T> {
    pub fn new(peripheral: T, config: UartConfig) -> Self {
        unsafe {
            let mut uart = Uart {
                rx: UartRx { peripheral },
                tx: UartTx {
                    peripheral: T::steal(),
                },
            };
            uart.set_config(&config).unwrap();
            init_uart_interrupt(uart.rx.peripheral.number() as usize);
            uart
        }
    }

    pub fn split(self) -> (UartRx<T>, UartTx<T>) {
        (self.rx, self.tx)
    }
}

impl<T: UartPeripheral> SetConfig for Uart<T> {
    type Config = UartConfig;
    type ConfigError = ();
    fn set_config(&mut self, config: &Self::Config) -> Result<(), ()> {
        unsafe {
            pac::MSS_UART_init(
                self.tx.peripheral.address(),
                config.baud_rate.value(),
                config.data_bits.value() | config.parity.value() | config.stop_bits.value(),
            );
        }
        Ok(())
    }
}

#[derive(Debug)]
pub enum Error {
    EmptyBuffer,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<T: UartPeripheral> embedded_io::ErrorType for Uart<T> {
    type Error = Error;
}

impl<T: UartPeripheral> embedded_io::Write for Uart<T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        self.tx.write(buf)
    }

    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

impl<T: UartPeripheral> embedded_io::Read for Uart<T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read(buf)
    }
}

impl<T: UartPeripheral> embedded_io_async::Read for Uart<T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read(buf).await
    }
}

//-----------------------------------------------------------------
// Tx

pub struct UartTx<T: UartPeripheral> {
    peripheral: T,
}

impl<TX: UartTxPeripheral> UartTx<TX> {
    pub fn new(peripheral: TX, config: UartConfig) -> Self {
        let mut uart = Self { peripheral };
        uart.set_config(&config).unwrap();
        uart
    }
}

impl<T: UartPeripheral> embedded_io::ErrorType for UartTx<T> {
    type Error = Error;
}

impl<T: UartPeripheral> embedded_io::Write for UartTx<T> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Error> {
        // TODO buffered writing
        unsafe {
            pac::MSS_UART_polled_tx(self.peripheral.address(), buf.as_ptr(), buf.len() as u32);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

impl<T: UartPeripheral> SetConfig for UartTx<T> {
    type Config = UartConfig;
    type ConfigError = ();
    fn set_config(&mut self, config: &Self::Config) -> Result<(), ()> {
        unsafe {
            pac::MSS_UART_init(
                self.peripheral.address(),
                config.baud_rate.value(),
                config.data_bits.value() | config.parity.value() | config.stop_bits.value(),
            );
        }
        Ok(())
    }
}

//-----------------------------------------------------------------
// Rx
const NUM_UARTS: usize = 5;
const RX_BUF_SIZE: usize = 128;

static mut RX_BUF: [[u8; RX_BUF_SIZE]; NUM_UARTS] = [[0; RX_BUF_SIZE]; NUM_UARTS];
// (write_idx, read_idx)
static mut RX_BUFFER_USED: [(usize, usize); NUM_UARTS] = [(0, 0); NUM_UARTS];
static mut RX_WAKERS: [Option<core::task::Waker>; NUM_UARTS] = [const { None }; NUM_UARTS];

pub struct UartRx<T: UartPeripheral> {
    peripheral: T,
}

impl<RX: UartRxPeripheral> UartRx<RX> {
    pub fn new(peripheral: RX, config: UartConfig) -> Self {
        let mut uart = Self { peripheral };
        uart.set_config(&config).unwrap();
        unsafe { init_uart_interrupt(uart.peripheral.number() as usize) };
        uart
    }
}

impl<T: UartPeripheral> SetConfig for UartRx<T> {
    type Config = UartConfig;
    type ConfigError = ();
    fn set_config(&mut self, config: &Self::Config) -> Result<(), ()> {
        unsafe {
            pac::MSS_UART_init(
                self.peripheral.address(),
                config.baud_rate.value(),
                config.data_bits.value() | config.parity.value() | config.stop_bits.value(),
            );
        }
        Ok(())
    }
}

impl<T: UartPeripheral> UartRx<T> {
    fn _read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let uart_idx = self.peripheral.number() as usize;
        let mut read = 0;

        while read < buf.len() {
            let (write_idx, read_idx) = critical_section::with(|_| unsafe {
                let indices = RX_BUFFER_USED[uart_idx];
                if indices.1 != indices.0 {
                    // Update read index
                    RX_BUFFER_USED[uart_idx].1 = (indices.1 + 1) % RX_BUF_SIZE;
                    // Copy data
                    buf[read] = RX_BUF[uart_idx][indices.1];
                }
                indices
            });

            if read_idx == write_idx {
                break;
            }
            read += 1;
        }

        Ok(read)
    }
}

impl<T: UartPeripheral> embedded_io::ErrorType for UartRx<T> {
    type Error = Error;
}

impl<T: UartPeripheral> embedded_io::Read for UartRx<T> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.len() == 0 {
            return Err(Error::EmptyBuffer);
        }
        unsafe {
            loop {
                let (write_idx, read_idx) = RX_BUFFER_USED[self.peripheral.number() as usize];
                if write_idx != read_idx {
                    break;
                }
            }
        }
        self._read(buf)
    }
}

impl<T: UartPeripheral> embedded_io_async::Read for UartRx<T> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.len() == 0 {
            return Err(Error::EmptyBuffer);
        }
        let uart_idx = self.peripheral.number() as usize;
        log::trace!("UART{}: read", uart_idx);

        // Wait for data if buffer is empty
        core::future::poll_fn(|cx| {
            critical_section::with(|_| unsafe {
                let (write_idx, read_idx) = RX_BUFFER_USED[uart_idx];
                if write_idx != read_idx {
                    Poll::Ready(())
                } else {
                    RX_WAKERS[uart_idx] = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await;

        self._read(buf)
    }
}

fn uart_idx(uart: *mut pac::mss_uart_instance_t) -> usize {
    if uart == &raw mut pac::g_mss_uart0_lo {
        0
    } else if uart == &raw mut pac::g_mss_uart1_lo {
        1
    } else if uart == &raw mut pac::g_mss_uart2_lo {
        2
    } else if uart == &raw mut pac::g_mss_uart3_lo {
        3
    } else if uart == &raw mut pac::g_mss_uart4_lo {
        4
    } else {
        panic!("Invalid UART instance")
    }
}

extern "C" fn uart_rx_handler(uart: *mut pac::mss_uart_instance_t) {
    let uart_idx = uart_idx(uart);
    log::trace!("UART{}: rx handler", uart_idx);
    unsafe {
        let (mut write_idx, mut read_idx) = RX_BUFFER_USED[uart_idx];

        // Read one byte into the circular buffer
        let size = pac::MSS_UART_get_rx(uart, &mut RX_BUF[uart_idx][write_idx] as *mut u8, 1);

        if size > 0 {
            // Update write index
            write_idx = (write_idx + 1) % RX_BUF_SIZE;

            // If buffer would overflow, advance read index
            if write_idx == read_idx {
                read_idx = (read_idx + 1) % RX_BUF_SIZE;
                RX_BUFFER_USED[uart_idx].1 = read_idx;
            }

            RX_BUFFER_USED[uart_idx].0 = write_idx;

            // Wake any waiting readers
            if let Some(waker) = RX_WAKERS[uart_idx].take() {
                waker.wake();
            }
        }

        log::trace!("UART{}: got {} bytes", uart_idx, size);
    }
}
