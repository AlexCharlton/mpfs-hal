use crate::pac;

pub trait UartPeripheral {
    fn address(&self) -> *mut pac::mss_uart_instance_t;
    fn number(&self) -> u8;
}

pub struct UART0 {}
static mut UART0_TAKEN: bool = false;

impl crate::Peripheral for UART0 {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if UART0_TAKEN {
                None
            } else {
                UART0_TAKEN = true;
                Some(Self {})
            }
        })
    }

    unsafe fn steal() -> Self {
        Self {}
    }
}

impl UartPeripheral for UART0 {
    fn address(&self) -> *mut pac::mss_uart_instance_t {
        core::ptr::addr_of_mut!(pac::g_mss_uart0_lo)
    }
    fn number(&self) -> u8 {
        0
    }
}

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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Error {
    InvalidParam,
    NoError,
    OverrunError,
    ParityError,
    FramingError,
    BreakError,
    FifoError,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

pub struct Uart<T: UartPeripheral> {
    peripheral: T,
}

impl<T: UartPeripheral> Uart<T> {
    pub fn new(peripheral: T, config: UartConfig) -> Self {
        critical_section::with(|_| unsafe {
            pac::mss_config_clk_rst(
                peripheral.number() as u32,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
            );
            pac::MSS_UART_init(
                peripheral.address(),
                config.baud_rate.value(),
                config.data_bits.value() | config.parity.value() | config.stop_bits.value(),
            );
        });
        Self { peripheral }
    }
}

impl<T: UartPeripheral> embedded_io::ErrorType for Uart<T> {
    type Error = Error;
}

impl<T: UartPeripheral> embedded_io::Write for Uart<T> {
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
