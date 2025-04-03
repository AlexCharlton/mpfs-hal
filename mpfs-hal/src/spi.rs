use crate::pac;
use embassy_embedded_hal::SetConfig;
use embedded_hal::spi::{Phase, Polarity};

pub fn init() {
    unsafe {
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_SPI0,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_SPI1,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        pac::MSS_SPI_init(&raw mut pac::g_mss_spi0_lo);
        pac::MSS_SPI_init(&raw mut pac::g_mss_spi1_lo);

        // pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_SPI0_INT_OFFSET, 2);
        // pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_SPI1_INT_OFFSET, 2);
        // pac::PLIC_EnableIRQ(pac::PLIC_IRQn_Type_PLIC_SPI0_INT_OFFSET);
        // pac::PLIC_EnableIRQ(pac::PLIC_IRQn_Type_PLIC_SPI1_INT_OFFSET);
    }
    log::debug!("SPI initialized");
}

pub trait SpiPeripheral: crate::Peripheral {
    #[doc(hidden)]
    fn address(&self) -> *mut pac::mss_spi_instance_t;
    #[doc(hidden)]
    fn number(&self) -> u8;
}

static mut SPI0_TAKEN: bool = false;
pub struct Spi0 {
    _private: (),
}

impl crate::Peripheral for Spi0 {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if SPI0_TAKEN {
                None
            } else {
                SPI0_TAKEN = true;
                Some(Self { _private: () })
            }
        })
    }

    unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

impl embedded_hal::spi::ErrorType for Spi0 {
    type Error = SpiError;
}

static mut SPI1_TAKEN: bool = false;
pub struct Spi1 {
    _private: (),
}

impl crate::Peripheral for Spi1 {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if SPI1_TAKEN {
                None
            } else {
                SPI1_TAKEN = true;
                Some(Self { _private: () })
            }
        })
    }

    unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

impl embedded_hal::spi::ErrorType for Spi1 {
    type Error = SpiError;
}

#[derive(Debug)]
pub enum SpiError {}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}
