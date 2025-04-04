use crate::{pac, Mutex};
use embassy_embedded_hal::SetConfig;
use embedded_hal::spi::{ErrorType, Operation, Phase, Polarity};
use paste::paste;

// Master mode only
// TODO: Add slave mode

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

        // Add for slave mode
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
    #[doc(hidden)]
    fn slave_num(&self) -> u8;
}

#[derive(Debug)]
pub enum SpiError {
    InvalidClockDiv,
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

static SPI0_IN_USE: Mutex = Mutex::new();
static SPI1_IN_USE: Mutex = Mutex::new();

macro_rules! impl_spi {
    ($n:expr, $slave_num:expr) => {
        paste! {
            impl_spi!([<Spi $n Slave $slave_num>], [<SPI $n _SLAVE $slave_num _TAKEN>], $n, $slave_num, [<g_mss_spi $n _lo>]);
        }
    };

    // E.g. impl_uart!(SPI0_SLAVE0, SPI0_SLAVE0_TAKEN, 0, 0, g_mss_spi_0_lo);
    ($PERIPH:ident, $PERIPH_TAKEN:ident, $spi_num:expr, $slave_num:expr, $instance:ident) => {
        pub struct $PERIPH {
            _private: (),
        }
        static mut $PERIPH_TAKEN: bool = false;

        impl crate::Peripheral for $PERIPH {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $PERIPH_TAKEN {
                        None
                    } else {
                        $PERIPH_TAKEN = true;
                        Some(Self { _private: () })
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }
        impl SpiPeripheral for $PERIPH {
            fn address(&self) -> *mut pac::mss_spi_instance_t {
                &raw mut pac::$instance
            }

            fn number(&self) -> u8 {
                $spi_num
            }

            fn slave_num(&self) -> u8 {
                $slave_num
            }
        }

    };
}

impl_spi!(0, 0);
impl_spi!(0, 1);
impl_spi!(1, 0);
impl_spi!(1, 1);

#[derive(Debug, Clone, Copy)]
pub struct SpiConfig {
    /// Clock divider, must be even between 2 to 512.
    /// The base clock is 150MHz, so the actual clock will be 150MHz / clock_div.
    /// Any value less than 8 results in instability on the CS line.
    clock_div: u16,
    phase: Phase,
    polarity: Polarity,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            clock_div: 8,
            phase: Phase::CaptureOnFirstTransition,
            polarity: Polarity::IdleLow,
        }
    }
}

impl SpiConfig {
    pub fn new(clock_div: u16, phase: Phase, polarity: Polarity) -> Result<Self, SpiError> {
        if clock_div < 2 || clock_div > 512 || clock_div % 2 != 0 {
            return Err(SpiError::InvalidClockDiv);
        }
        Ok(Self {
            clock_div,
            phase,
            polarity,
        })
    }
}

pub struct Spi<T: SpiPeripheral> {
    peripheral: T,
}

impl<T: SpiPeripheral> ErrorType for Spi<T> {
    type Error = SpiError;
}

impl<T: SpiPeripheral> Spi<T> {
    pub fn new(peripheral: T, config: SpiConfig) -> Self {
        let mut spi = Spi { peripheral };
        spi.set_config(&config).unwrap();
        spi
    }
}

impl<T: SpiPeripheral> SetConfig for Spi<T> {
    type Config = SpiConfig;
    type ConfigError = SpiError;
    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        let mode = match (config.phase, config.polarity) {
            (Phase::CaptureOnFirstTransition, Polarity::IdleLow) => {
                pac::__mss_spi_protocol_mode_t_MSS_SPI_MODE0
            }
            (Phase::CaptureOnSecondTransition, Polarity::IdleLow) => {
                pac::__mss_spi_protocol_mode_t_MSS_SPI_MODE1
            }
            (Phase::CaptureOnFirstTransition, Polarity::IdleHigh) => {
                pac::__mss_spi_protocol_mode_t_MSS_SPI_MODE2
            }
            (Phase::CaptureOnSecondTransition, Polarity::IdleHigh) => {
                pac::__mss_spi_protocol_mode_t_MSS_SPI_MODE3
            }
        };
        unsafe {
            pac::MSS_SPI_configure_master_mode(
                self.peripheral.address(),
                self.peripheral.slave_num() as u32,
                mode,
                config.clock_div as u32,
                pac::MSS_SPI_BLOCK_TRANSFER_FRAME_SIZE as u8, // This is ignored
                Some(mss_spi_overflow_handler),
            );
        }
        Ok(())
    }
}

extern "C" fn mss_spi_overflow_handler(mss_spi_core: u8) {
    unsafe {
        if mss_spi_core != 0 {
            // reset SPI1
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI1,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_OFF,
            );
            // Take SPI1 out of reset
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI1,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
            );
        } else {
            // reset SPI0
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI0,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_OFF,
            );
            // Take SPI0 out of reset
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI0,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
            );
        }
    }
}

impl<T: SpiPeripheral> embedded_hal::spi::SpiDevice for Spi<T> {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let mutex = if self.peripheral.number() == 0 {
            &SPI0_IN_USE
        } else {
            &SPI1_IN_USE
        };
        let lock = mutex.lock();

        unsafe {
            pac::MSS_SPI_set_slave_select(
                &raw mut pac::g_mss_spi0_lo,
                pac::__mss_spi_slave_t_MSS_SPI_SLAVE_0,
            );
            for operation in operations {
                match operation {
                    Operation::Write(data) => {
                        self.write(data)?;
                    }
                    Operation::Read(data) => {
                        self.read(data)?;
                    }
                    Operation::Transfer(rx, tx) => {
                        self.transfer(rx, tx)?;
                    }
                    Operation::TransferInPlace(data) => {
                        self.transfer_in_place(data)?;
                    }
                    Operation::DelayNs(ns) => {
                        let now = pac::readmcycle();
                        // Convert ns to ticks: ns * (600 MHz / 1000 MHz) = ns * 0.6
                        // Round up to ensure we wait at least the requested time
                        let ticks = (*ns * 3 + 4) / 5; // Equivalent to ceil(ns * 0.6)
                        while pac::readmcycle() < now + ticks as u64 {
                            core::hint::spin_loop();
                        }
                    }
                }
            }
            pac::MSS_SPI_clear_slave_select(
                &raw mut pac::g_mss_spi0_lo,
                pac::__mss_spi_slave_t_MSS_SPI_SLAVE_0,
            );
        }
        mutex.release(lock);

        Ok(())
    }
}

impl<T: SpiPeripheral> Spi<T> {
    fn write(&mut self, data: &[u8]) -> Result<(), SpiError> {
        if data.is_empty() {
            return Ok(());
        }
        let buffer_aligned: bool = data.as_ptr().align_offset(4) == 0 && data.len() > 3;
        log::debug!(
            "Writing to SPI {:x?}. Buffer aligned: {}; Byte count: {}",
            data,
            buffer_aligned,
            data.len()
        );

        unsafe {
            let spi = &mut (*(*self.peripheral.address()).hw_reg);

            let word_count = if buffer_aligned { data.len() / 4 } else { 0 } as u32;
            if word_count > 0 {
                // Transfer 32-bit aligned words
                let words = data.as_ptr() as *const u32;

                spi.FRAMESIZE = 32;
                spi.FRAMESUP = word_count as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | ((word_count << pac::spi::TXRXDFCOUNT_SHIFT) & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..(word_count as usize) {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    spi.TX_DATA = (*words.add(i)).to_be();
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }

            let data = &data[word_count as usize * 4..];

            if !data.is_empty() {
                spi.FRAMESIZE = 8;
                spi.FRAMESUP = data.len() as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | (((data.len() as u32) << pac::spi::TXRXDFCOUNT_SHIFT)
                        & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..data.len() {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    spi.TX_DATA = data[i] as u32;
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }
        }
        Ok(())
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), SpiError> {
        if data.is_empty() {
            return Ok(());
        }
        let buffer_aligned: bool = data.as_ptr().align_offset(4) == 0 && data.len() > 3;

        log::debug!(
            "Reading from SPI. Buffer aligned: {}; Byte count: {}",
            buffer_aligned,
            data.len()
        );

        unsafe {
            let spi = &mut (*(*self.peripheral.address()).hw_reg);
            let word_count = if buffer_aligned { data.len() / 4 } else { 0 } as u32;
            if word_count > 0 {
                let words = data.as_ptr() as *mut u32;

                spi.FRAMESIZE = 32;
                spi.FRAMESUP = word_count as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | ((word_count << pac::spi::TXRXDFCOUNT_SHIFT) & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..(word_count as usize) {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    spi.TX_DATA = 0xFFFFFFFF;
                    while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    *words.add(i) = spi.RX_DATA.to_be();
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }

            let data = &mut data[word_count as usize * 4..];

            if !data.is_empty() {
                spi.FRAMESIZE = 8;
                spi.FRAMESUP = data.len() as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | (((data.len() as u32) << pac::spi::TXRXDFCOUNT_SHIFT)
                        & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..data.len() {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    spi.TX_DATA = 0xFFFFFFFF;
                    while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    data[i] = spi.RX_DATA as u8;
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }
        }

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), SpiError> {
        if read.is_empty() && write.is_empty() {
            return Ok(());
        }
        let buffer_aligned: bool = write.as_ptr().align_offset(4) == 0
            && read.as_ptr().align_offset(4) == 0
            && (write.len() > 3 || read.len() > 3);

        log::debug!(
            "Transferring from SPI. Read len: {}; Write: {:x?}; Buffer aligned: {}",
            read.len(),
            write,
            buffer_aligned
        );

        let total_bytes = read.len().max(write.len());
        let mut tx_bytes_sent = 0;
        let mut rx_bytes_received = 0;

        unsafe {
            let spi = &mut (*(*self.peripheral.address()).hw_reg);
            let word_count = if buffer_aligned { total_bytes / 4 } else { 0 } as u32;
            if word_count > 0 {
                let tx_words = write.as_ptr() as *mut u32;
                let rx_words = read.as_ptr() as *mut u32;

                spi.FRAMESIZE = 32;
                spi.FRAMESUP = word_count as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | ((word_count << pac::spi::TXRXDFCOUNT_SHIFT) & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..(word_count as usize) {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    if tx_bytes_sent + 4 <= write.len() {
                        spi.TX_DATA = (*tx_words.add(i)).to_be();
                        tx_bytes_sent += 4;
                    } else if tx_bytes_sent < write.len() {
                        let mut tx_data = 0x0;
                        let tx_bytes_remaining = write.len() - tx_bytes_sent;
                        while tx_bytes_sent < write.len() {
                            tx_data <<= 8;
                            tx_data |= write[tx_bytes_sent] as u32;
                            tx_bytes_sent += 1;
                        }
                        for _ in 0..(4 - tx_bytes_remaining) {
                            tx_data <<= 8;
                            tx_data |= 0xFF;
                        }
                        spi.TX_DATA = tx_data;
                    } else {
                        spi.TX_DATA = 0xFFFFFFFF;
                    }
                    while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    let word = spi.RX_DATA;
                    if rx_bytes_received + 4 <= read.len() {
                        *rx_words.add(i) = word.to_be();
                        rx_bytes_received += 4;
                    } else if rx_bytes_received < read.len() {
                        let rx_bytes_remaining = read.len() - rx_bytes_received;
                        for i in 0..(4 - rx_bytes_remaining) {
                            read[rx_bytes_received] = (word >> (8 * i)) as u8;
                            rx_bytes_received += 1;
                        }
                    }
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }

            let write = &write[tx_bytes_sent..];
            let read = &mut read[rx_bytes_received..];
            let remaining_bytes = read.len().max(write.len());

            if remaining_bytes > 0 {
                spi.FRAMESIZE = 8;
                spi.FRAMESUP = remaining_bytes as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | (((remaining_bytes as u32) << pac::spi::TXRXDFCOUNT_SHIFT)
                        & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..remaining_bytes {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    if i < write.len() {
                        spi.TX_DATA = write[i] as u32;
                    }
                    while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    if i < read.len() {
                        read[i] = spi.RX_DATA as u8;
                    }
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }
        }

        Ok(())
    }

    fn transfer_in_place(&mut self, data: &mut [u8]) -> Result<(), SpiError> {
        if data.is_empty() {
            return Ok(());
        }

        let buffer_aligned: bool = data.as_ptr().align_offset(4) == 0 && data.len() > 3;
        log::debug!(
            "Transferring in place, buffer aligned? {}; Data: {:x?}",
            buffer_aligned,
            data
        );

        unsafe {
            let spi = &mut (*(*self.peripheral.address()).hw_reg);
            let word_count = if buffer_aligned { data.len() / 4 } else { 0 } as u32;

            if word_count > 0 {
                let words = data.as_ptr() as *mut u32;

                spi.FRAMESIZE = 32;
                spi.FRAMESUP = word_count as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | ((word_count << pac::spi::TXRXDFCOUNT_SHIFT) & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..(word_count as usize) {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    spi.TX_DATA = (*words.add(i)).to_be();
                    while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    *words.add(i) = spi.RX_DATA.to_be();
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }

            let data = &mut data[word_count as usize * 4..];

            if !data.is_empty() {
                spi.FRAMESIZE = 8;
                spi.FRAMESUP = data.len() as u32 & pac::spi::BYTESUPPER_MASK;
                spi.CONTROL = (spi.CONTROL & !pac::spi::TXRXDFCOUNT_MASK)
                    | (((data.len() as u32) << pac::spi::TXRXDFCOUNT_SHIFT)
                        & pac::spi::TXRXDFCOUNT_MASK);
                spi.CONTROL |= pac::spi::CTRL_ENABLE_MASK;
                // Flush the receive FIFO
                while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK == 0 {
                    let _ = spi.RX_DATA;
                }

                for i in 0..data.len() {
                    // Wait until transmit FIFO is not full
                    while spi.STATUS & pac::spi::TX_FIFO_FULL_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    spi.TX_DATA = data[i] as u32;
                    while spi.STATUS & pac::spi::RX_FIFO_EMPTY_MASK != 0 {
                        core::hint::spin_loop();
                    }
                    data[i] = spi.RX_DATA as u8;
                }

                // Wait until the transfer is done
                while spi.STATUS & pac::spi::ACTIVE_MASK != 0 {
                    core::hint::spin_loop();
                }

                // Flush the FIFOs
                spi.COMMAND |= pac::spi::TX_FIFO_RESET_MASK | pac::spi::RX_FIFO_RESET_MASK;
                // Disable the SPI
                spi.CONTROL &= !pac::spi::CTRL_ENABLE_MASK;
            }
        }

        Ok(())
    }
}
