use crate::pac;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Peripheral {
    Uart0,
    Uart1,
    Uart2,
    Uart3,
    Uart4,
}

impl Peripheral {
    fn address(&self) -> *mut pac::mss_uart_instance_t {
        match self {
            Peripheral::Uart0 => core::ptr::addr_of_mut!(pac::g_mss_uart0_lo),
            Peripheral::Uart1 => core::ptr::addr_of_mut!(pac::g_mss_uart1_lo),
            Peripheral::Uart2 => core::ptr::addr_of_mut!(pac::g_mss_uart2_lo),
            Peripheral::Uart3 => core::ptr::addr_of_mut!(pac::g_mss_uart3_lo),
            Peripheral::Uart4 => core::ptr::addr_of_mut!(pac::g_mss_uart4_lo),
        }
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DataBits {
    Data5,
    Data6,
    Data7,
    Data8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StopBits {
    One,
    OneHalf,
    Two,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Parity {
    NoParity,
    OddParity,
    EvenParity,
    StickParity0,
    StickParity1,
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

pub struct Uart {
    peripheral: Peripheral,
    config: UartConfig,
}

impl Uart {
    pub fn new(peripheral: Peripheral, config: UartConfig) -> Self {
        // TODO Init
        Self { peripheral, config }
    }
}

impl embedded_io::ErrorType for Uart {
    type Error = Error;
}

impl embedded_io::Write for Uart {
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
