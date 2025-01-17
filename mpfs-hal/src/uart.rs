use crate::pac;
use embassy_embedded_hal::SetConfig;
use paste::paste;

pub trait UartPeripheral {
    fn address(&self) -> *mut pac::mss_uart_instance_t;
    fn number(&self) -> u8;
}

//-------------------------------------------------------------------
// Create the UART peripherals

macro_rules! impl_uart {
    ($n:expr) => {
        paste! {
            impl_uart!([<UART $n>], [<UART $n _TAKEN>], $n, [<g_mss_uart $n _lo>]);
        }
    };

    // E.g. impl_uart!(UART0, UART0_TAKEN, 0, g_mss_uart_0_lo);
    ($UART:ident, $UART_TAKEN:ident, $num:expr, $instance:ident) => {
        pub struct $UART {
            _private: (),
        }
        static mut $UART_TAKEN: bool = false;

        impl crate::Peripheral for $UART {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $UART_TAKEN {
                        None
                    } else {
                        $UART_TAKEN = true;
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
                core::ptr::addr_of_mut!(pac::$instance)
            }

            fn number(&self) -> u8 {
                $num
            }
        }
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
            let mut uart = Uart { peripheral };
            uart.set_config(&config).unwrap();
            uart
        })
    }
}

impl<T: UartPeripheral> SetConfig for Uart<T> {
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

#[derive(Debug)]
pub enum Error {}

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

// TODO: Implement Read
