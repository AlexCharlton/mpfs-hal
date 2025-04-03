use super::bindings::*;

pub const QSPI: *mut QSPI_TypeDef = QSPI_BASE as *mut QSPI_TypeDef;
pub const SPI0: *mut SPI_TypeDef = SPI0_LO_BASE as *mut SPI_TypeDef;
pub const SPI1: *mut SPI_TypeDef = SPI1_LO_BASE as *mut SPI_TypeDef;
