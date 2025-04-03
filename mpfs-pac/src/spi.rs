use super::bindings::*;

pub const QSPI: *mut QSPI_TypeDef = QSPI_BASE as *mut QSPI_TypeDef;
pub const SPI0: *mut SPI_TypeDef = SPI0_LO_BASE as *mut SPI_TypeDef;
pub const SPI1: *mut SPI_TypeDef = SPI1_LO_BASE as *mut SPI_TypeDef;

/// Mask of transfer protocol and SPO, SPH bits within control register.
pub const PROTOCOL_MODE_MASK: u32 = 0x0300_000C;

/***************************************************************************/
/**
 * Mask of the frame count bits within the SPI control register.
 */
/// Mask of the frame count bits within the SPI control register.
pub const TXRXDFCOUNT_MASK: u32 = 0x00FF_FF00;
/// Shift of the frame count bits within the SPI control register.
pub const TXRXDFCOUNT_SHIFT: u32 = 8;
/// Mask of the upper byte of the frame count within the SPI control register.
pub const BYTESUPPER_MASK: u32 = 0xFFFF_0000;
/***************************************************************************/
/**
 * SPI hardware FIFO depth.
 */
/// SPI hardware FIFO depth.
pub const RX_FIFO_SIZE: u32 = 4;
/// SPI hardware FIFO depth.
pub const BIG_FIFO_SIZE: u32 = 32;

/***************************************************************************/
/**
 * CONTROL register bit masks
 */
/// CONTROL register bit masks
pub const CTRL_ENABLE_MASK: u32 = 0x0000_0001;
/// CONTROL register bit masks
pub const CTRL_MASTER_MASK: u32 = 0x0000_0002;

/***************************************************************************/
/**
 Registers bit masks
*/
/* CONTROL register. */
/// CONTROL register bit masks
pub const MASTER_MODE_MASK: u32 = 0x0000_0002;
/// CONTROL register bit masks
pub const CTRL_RX_IRQ_EN_MASK: u32 = 0x0000_0010;
/// CONTROL register bit masks
pub const CTRL_TX_IRQ_EN_MASK: u32 = 0x0000_0020;
/// CONTROL register bit masks
pub const CTRL_OVFLOW_IRQ_EN_MASK: u32 = 0x0000_0040;
/// CONTROL register bit masks
pub const CTRL_URUN_IRQ_EN_MASK: u32 = 0x0000_0080;
/// CONTROL register bit masks
pub const CTRL_REG_RESET_MASK: u32 = 0x8000_0000;
/// CONTROL register bit masks
pub const BIGFIFO_MASK: u32 = 0x2000_0000;
/// CONTROL register bit masks
pub const CTRL_CLKMODE_MASK: u32 = 0x1000_0000;
/// CONTROL register bit masks
pub const SPS_MASK: u32 = 0x0400_0000;

/* CONTROL2 register */
/// CONTROL2 register bit masks
pub const C2_ENABLE_CMD_IRQ_MASK: u32 = 0x0000_0010;
/// CONTROL2 register bit masks
pub const C2_ENABLE_SSEND_IRQ_MASK: u32 = 0x0000_0020;

/* STATUS register */
/// STATUS register bit masks
pub const TX_DONE_MASK: u32 = 0x0000_0001;
/// STATUS register bit masks
pub const RX_DATA_READY_MASK: u32 = 0x0000_0002;
/// STATUS register bit masks
pub const RX_OVERFLOW_MASK: u32 = 0x0000_0004;
/// STATUS register bit masks
pub const RX_FIFO_EMPTY_MASK: u32 = 0x0000_0040;
/// STATUS register bit masks
pub const TX_FIFO_FULL_MASK: u32 = 0x0000_0100;
/// STATUS register bit masks
pub const TX_FIFO_EMPTY_MASK: u32 = 0x0000_0400;
/// ACTIVE: SPI is still transmitting or receiving data.
pub const ACTIVE_MASK: u32 = 0x0000_4000;

/* MIS register. */
/// MIS register bit masks
pub const TXDONE_IRQ_MASK: u32 = 0x0000_0001;
/// MIS register bit masks
pub const RXDONE_IRQ_MASK: u32 = 0x0000_0002;
/// MIS register bit masks
pub const RXOVFLOW_IRQ_MASK: u32 = 0x0000_0004;
/// MIS register bit masks
pub const TXURUN_IRQ_MASK: u32 = 0x0000_0008;
/// MIS register bit masks
pub const CMD_IRQ_MASK: u32 = 0x0000_0010;
/// MIS register bit masks
pub const SSEND_IRQ_MASK: u32 = 0x0000_0020;

/* COMMAND register */
/// COMMAND register bit masks
pub const AUTOFILL_MASK: u32 = 0x0000_0001;
/// COMMAND register bit masks
pub const TX_FIFO_RESET_MASK: u32 = 0x0000_0008;
/// COMMAND register bit masks
pub const RX_FIFO_RESET_MASK: u32 = 0x0000_0004;
