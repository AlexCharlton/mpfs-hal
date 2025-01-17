use crate::pac;
use embassy_embedded_hal::SetConfig;
use embedded_hal::spi::{Phase, Polarity};

static mut QSPI_TAKEN: bool = false;

pub fn init() {
    unsafe {
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_CFM,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_QSPIXIP,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        pac::MSS_QSPI_init();
    }
}

pub struct Qspi {
    _private: (),
}

impl crate::Peripheral for Qspi {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if QSPI_TAKEN {
                None
            } else {
                QSPI_TAKEN = true;
                Some(Self { _private: () })
            }
        })
    }

    unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

impl embedded_hal::spi::ErrorType for Qspi {
    type Error = SpiError;
}

#[derive(Debug)]
pub enum SpiError {}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SpiConfig {
    pub frequency: SpiFrequency,
    pub phase: Phase,
    pub polarity: Polarity,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            frequency: SpiFrequency::F5_000_000,
            phase: Phase::CaptureOnFirstTransition,
            polarity: Polarity::IdleLow,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum SpiFrequency {
    F75_000_000 = 1,  // 150MHz / 2
    F37_500_000 = 2,  // 150MHz / 4
    F25_000_000 = 3,  // 150MHz / 6
    F18_750_000 = 4,  // 150MHz / 8
    F15_000_000 = 5,  // 150MHz / 10
    F12_500_000 = 6,  // 150MHz / 12
    F10_714_285 = 7,  // 150MHz / 14
    F9_375_000 = 8,   // 150MHz / 16
    F8_333_333 = 9,   // 150MHz / 18
    F7_500_000 = 0xA, // 150MHz / 20
    F6_818_181 = 0xB, // 150MHz / 22
    F6_250_000 = 0xC, // 150MHz / 24
    F5_769_230 = 0xD, // 150MHz / 26
    F5_357_142 = 0xE, // 150MHz / 28
    F5_000_000 = 0xF, // 150MHz / 30
}

impl SetConfig for Qspi {
    type Config = SpiConfig;
    type ConfigError = ();
    fn set_config(&mut self, config: &Self::Config) -> Result<(), ()> {
        unsafe {
            let sample = if config.phase == Phase::CaptureOnFirstTransition {
                pac::MSS_QSPI_SAMPLE_POSAGE_SPICLK
            } else {
                pac::MSS_QSPI_SAMPLE_NEGAGE_SPICLK
            };
            let mode = if config.polarity == Polarity::IdleLow {
                pac::mss_qspi_protocol_mode_t_MSS_QSPI_MODE0
            } else {
                pac::mss_qspi_protocol_mode_t_MSS_QSPI_MODE3
            };
            let value = (sample << pac::CTRL_SAMPLE)
                | ((config.frequency as u32) << pac::CTRL_CLKRATE)
                | (pac::mss_qspi_io_format_t_MSS_QSPI_NORMAL << pac::CTRL_QMODE12)
                | (mode << pac::CTRL_CLKIDL)
                | (0 << pac::CTRL_XIP)
                | (0 << pac::CTRL_XIPADDR)
                | pac::CTRL_EN_MASK;
            log::trace!("QSPI Control Register: {:#032b}", value);
            (*pac::QSPI).CONTROL = value;
        }
        Ok(())
    }
}

/// When buffers are 4 byte aligned, QSPI can read and write 32-bit words. Otherwise, the 8-bit
/// interface is used.
impl embedded_hal::spi::SpiBus<u8> for Qspi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let buffer_aligned: bool = words.as_ptr().align_offset(4) == 0 && words.len() > 3;
        log::debug!("Reading from QSPI. Buffer aligned: {:?}", buffer_aligned);
        unsafe {
            // Wait until QSPI is ready
            while ((*pac::QSPI).STATUS & pac::STTS_READY_MASK) == 0 {
                core::hint::spin_loop();
            }

            // Disable interrupts
            (*pac::QSPI).INTENABLE = 0;

            // Configure frame size
            let total_bytes = words.len();
            (*pac::QSPI).FRAMESUP = (total_bytes as u32) & pac::FRMS_UBYTES_MASK;

            // Configure frame control
            let mut frame_ctrl = (total_bytes as u32) & 0xFFFF;
            frame_ctrl |= 0 << pac::FRMS_CBYTES; // Set command bytes to 0, which will write and read whenever something is in the TX FIFO
            frame_ctrl |= ((*pac::QSPI).CONTROL & pac::CTRL_QMODE12_MASK) << pac::FRMS_QSPI; // If set to QSPI mode, set QSPI bit

            frame_ctrl |= if buffer_aligned {
                pac::FRMS_FWORD_MASK
            } else {
                pac::FRMS_FBYTE_MASK
            };
            (*pac::QSPI).FRAMES = frame_ctrl;

            let word_count = if buffer_aligned { total_bytes / 4 } else { 0 };
            if buffer_aligned {
                // Enable 32-bit transfer mode
                (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

                // Transfer 32-bit aligned words
                let words_32 = words.as_ptr() as *mut u32;

                for i in 0..word_count {
                    // Wait until transmit FIFO is not full
                    while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                        core::hint::spin_loop();
                    }
                    // Send dummy data
                    (*pac::QSPI).TXDATAX4 = 0xFFFFFFFF;

                    // Wait until receive FIFO is not empty
                    while ((*pac::QSPI).STATUS & pac::STTS_RFEMPTY_MASK) != 0 {
                        core::hint::spin_loop();
                    }
                    *words_32.add(i) = (*pac::QSPI).RXDATAX4;
                }

                // Disable 32-bit transfer mode
                (*pac::QSPI).CONTROL &= !pac::CTRL_FLAGSX4_MASK;

                if total_bytes % 4 != 0 {
                    // Without this delay the TXDATAX1 FIFO does not get updated with proper data
                    pac::sleep_ms(10);
                }
            }

            // Transfer remaining bytes
            let remaining_start = word_count * 4;
            for i in remaining_start..total_bytes {
                // Wait until transmit FIFO is not full
                while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                    core::hint::spin_loop();
                }
                // Send dummy data
                (*pac::QSPI).TXDATAX1 = 0xFF;

                // Wait until receive FIFO is not empty
                while ((*pac::QSPI).STATUS & pac::STTS_RFEMPTY_MASK) != 0 {
                    core::hint::spin_loop();
                }
                words[i] = (*pac::QSPI).RXDATAX1;
            }
            // Make sure the read is complete (but we shouldn't get here)
            while ((*pac::QSPI).STATUS & pac::STTS_RDONE_MASK) == 0 {
                log::warn!("Warning: read not complete");
                if ((*pac::QSPI).STATUS & pac::STTS_FLAGSX4_MASK) != 0 {
                    (*pac::QSPI).RXDATAX4;
                } else {
                    (*pac::QSPI).RXDATAX1;
                }
            }
        }
        log::debug!("QSPI read complete: {:x?}", words);
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let buffer_aligned: bool = words.as_ptr().align_offset(4) == 0 && words.len() > 3;
        log::debug!(
            "Writing to QSPI {:x?}. Buffer aligned: {:?}",
            words,
            buffer_aligned
        );
        unsafe {
            // Wait until QSPI is ready
            while ((*pac::QSPI).STATUS & pac::STTS_READY_MASK) == 0 {
                core::hint::spin_loop();
            }

            // Disable interrupts
            (*pac::QSPI).INTENABLE = 0;

            // Configure frame size
            let total_bytes = words.len();
            (*pac::QSPI).FRAMESUP = (total_bytes as u32) & pac::FRMS_UBYTES_MASK;

            // Configure frame control
            let mut frame_ctrl = (total_bytes as u32) & 0xFFFF;
            frame_ctrl |= 0 << pac::FRMS_CBYTES; // Set command bytes to 0, which will write and read whenever something is in the TX FIFO
            frame_ctrl |= ((*pac::QSPI).CONTROL & pac::CTRL_QMODE12_MASK) << pac::FRMS_QSPI; // If set to QSPI mode, set QSPI bit

            frame_ctrl |= pac::FRMS_FWORD_MASK; // Set full word mode
            (*pac::QSPI).FRAMES = frame_ctrl;

            let word_count = if buffer_aligned { total_bytes / 4 } else { 0 };
            if buffer_aligned {
                // Enable 32-bit transfer mode
                (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

                // Transfer 32-bit aligned words
                let words_32 = words.as_ptr() as *const u32;

                for i in 0..word_count {
                    // Wait until transmit FIFO is not full
                    while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                        core::hint::spin_loop();
                    }
                    (*pac::QSPI).TXDATAX4 = *words_32.add(i);
                }

                // Disable 32-bit transfer mode
                (*pac::QSPI).CONTROL &= !pac::CTRL_FLAGSX4_MASK;

                if total_bytes % 4 != 0 {
                    // Without this delay the TXDATAX1 FIFO does not get updated with proper data
                    pac::sleep_ms(10);
                }
            }

            // Transfer remaining bytes
            let remaining_start = word_count * 4;
            for i in remaining_start..total_bytes {
                // Wait until transmit FIFO is not full
                while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                    core::hint::spin_loop();
                }
                (*pac::QSPI).TXDATAX1 = words[i];
            }
            while ((*pac::QSPI).STATUS & pac::STTS_TDONE_MASK) == 0 {
                core::hint::spin_loop();
            }
        }
        log::debug!("QSPI write complete");
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let buffer_aligned: bool = write.as_ptr().align_offset(4) == 0
            && read.as_ptr().align_offset(4) == 0
            && (write.len() > 3 || read.len() > 3);
        log::debug!(
            "QSPI transfer {:x?}. Buffer aligned: {:?}",
            write,
            buffer_aligned
        );
        unsafe {
            // Wait until QSPI is ready
            while ((*pac::QSPI).STATUS & pac::STTS_READY_MASK) == 0 {
                core::hint::spin_loop();
            }

            // Disable interrupts
            (*pac::QSPI).INTENABLE = 0;

            // Configure frame size
            let total_bytes = read.len().max(write.len());
            (*pac::QSPI).FRAMESUP = (total_bytes as u32) & pac::FRMS_UBYTES_MASK;

            // Configure frame control
            let mut frame_ctrl = (total_bytes as u32) & 0xFFFF;
            frame_ctrl |= 0 << pac::FRMS_CBYTES; // Set command bytes to 0, which will write and read whenever something is in the TX FIFO
            frame_ctrl |= ((*pac::QSPI).CONTROL & pac::CTRL_QMODE12_MASK) << pac::FRMS_QSPI; // If set to QSPI mode, set QSPI bit

            frame_ctrl |= if buffer_aligned {
                pac::FRMS_FWORD_MASK
            } else {
                pac::FRMS_FBYTE_MASK
            };
            (*pac::QSPI).FRAMES = frame_ctrl;

            // Enable 32-bit transfer mode
            (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

            // Transfer 32-bit aligned words
            let write_32 = write.as_ptr() as *const u32;
            let read_32 = read.as_ptr() as *mut u32;
            let word_count = if buffer_aligned { total_bytes / 4 } else { 0 };
            let tx_word_count = write.len() / 4;
            let rx_word_count = read.len() / 4;
            if buffer_aligned {
                for i in 0..word_count {
                    // Wait until transmit FIFO is not full
                    while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                        core::hint::spin_loop();
                    }
                    if i < tx_word_count {
                        // Write data
                        (*pac::QSPI).TXDATAX4 = *write_32.add(i);
                    } else {
                        // Send dummy data
                        (*pac::QSPI).TXDATAX4 = 0xFFFFFFFF;
                    }

                    // Wait until receive FIFO is not empty
                    while ((*pac::QSPI).STATUS & pac::STTS_RFEMPTY_MASK) != 0 {
                        core::hint::spin_loop();
                    }
                    if i < rx_word_count {
                        *read_32.add(i) = (*pac::QSPI).RXDATAX4;
                    } else {
                        (*pac::QSPI).RXDATAX1; // discard
                    }
                }

                // Disable 32-bit transfer mode
                (*pac::QSPI).CONTROL &= !pac::CTRL_FLAGSX4_MASK;

                if total_bytes % 4 != 0 {
                    // Without this delay the TXDATAX1 FIFO does not get updated with proper data
                    pac::sleep_ms(10);
                }
            }

            // Transfer remaining bytes
            let remaining_start = word_count * 4;
            for i in remaining_start..total_bytes {
                // Wait until transmit FIFO is not full
                while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                    core::hint::spin_loop();
                }
                if i < write.len() {
                    // Write data
                    (*pac::QSPI).TXDATAX1 = write[i];
                } else {
                    // Send dummy data
                    (*pac::QSPI).TXDATAX1 = 0xFF;
                }

                // Wait until receive FIFO is not empty
                while ((*pac::QSPI).STATUS & pac::STTS_RFEMPTY_MASK) != 0 {
                    core::hint::spin_loop();
                }
                if i < read.len() {
                    read[i] = (*pac::QSPI).RXDATAX1;
                } else {
                    (*pac::QSPI).RXDATAX1; // discard
                }
            }
            // Make sure the read is complete (but we shouldn't get here)
            while ((*pac::QSPI).STATUS & pac::STTS_RDONE_MASK) == 0 {
                log::warn!("Warning: read not complete");
                if ((*pac::QSPI).STATUS & pac::STTS_FLAGSX4_MASK) != 0 {
                    (*pac::QSPI).RXDATAX4;
                } else {
                    (*pac::QSPI).RXDATAX1;
                }
            }
        }
        log::debug!("QSPI transfer received {:x?}", read);
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let buffer_aligned: bool = words.as_ptr().align_offset(4) == 0 && words.len() > 3;
        log::debug!(
            "QSPI transfer_in_place {:x?}. Buffer aligned: {:?}",
            words,
            buffer_aligned
        );
        unsafe {
            // Wait until QSPI is ready
            while ((*pac::QSPI).STATUS & pac::STTS_READY_MASK) == 0 {
                core::hint::spin_loop();
            }

            // Disable interrupts
            (*pac::QSPI).INTENABLE = 0;

            // Configure frame size
            let total_bytes = words.len();
            (*pac::QSPI).FRAMESUP = (total_bytes as u32) & pac::FRMS_UBYTES_MASK;

            // Configure frame control
            let mut frame_ctrl = (total_bytes as u32) & 0xFFFF;
            frame_ctrl |= 0 << pac::FRMS_CBYTES; // Set command bytes to 0, which will write and read whenever something is in the TX FIFO
            frame_ctrl |= ((*pac::QSPI).CONTROL & pac::CTRL_QMODE12_MASK) << pac::FRMS_QSPI; // If set to QSPI mode, set QSPI bit

            frame_ctrl |= if buffer_aligned {
                pac::FRMS_FWORD_MASK
            } else {
                pac::FRMS_FBYTE_MASK
            };
            (*pac::QSPI).FRAMES = frame_ctrl;

            let word_count = if buffer_aligned { total_bytes / 4 } else { 0 };
            if buffer_aligned {
                // Enable 32-bit transfer mode
                (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

                // Transfer 32-bit aligned words
                let words_32 = words.as_ptr() as *mut u32;

                for i in 0..word_count {
                    // Wait until transmit FIFO is not full
                    while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                        core::hint::spin_loop();
                    }
                    (*pac::QSPI).TXDATAX4 = *words_32.add(i);

                    // Wait until receive FIFO is not empty
                    while ((*pac::QSPI).STATUS & pac::STTS_RFEMPTY_MASK) != 0 {
                        core::hint::spin_loop();
                    }
                    *words_32.add(i) = (*pac::QSPI).RXDATAX4;
                }

                // Disable 32-bit transfer mode
                (*pac::QSPI).CONTROL &= !pac::CTRL_FLAGSX4_MASK;

                if total_bytes % 4 != 0 {
                    // Without this delay the TXDATAX1 FIFO does not get updated with proper data
                    pac::sleep_ms(10);
                }
            }

            // Transfer remaining bytes
            let remaining_start = word_count * 4;
            for i in remaining_start..total_bytes {
                // Wait until transmit FIFO is not full
                while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                    core::hint::spin_loop();
                }
                (*pac::QSPI).TXDATAX1 = words[i];

                // Wait until receive FIFO is not empty
                while ((*pac::QSPI).STATUS & pac::STTS_RFEMPTY_MASK) != 0 {
                    core::hint::spin_loop();
                }
                words[i] = (*pac::QSPI).RXDATAX1;
            }
            // Make sure the read is complete (but we shouldn't get here)
            while ((*pac::QSPI).STATUS & pac::STTS_RDONE_MASK) == 0 {
                log::warn!("Warning: read not complete");
                if ((*pac::QSPI).STATUS & pac::STTS_FLAGSX4_MASK) != 0 {
                    (*pac::QSPI).RXDATAX4;
                } else {
                    (*pac::QSPI).RXDATAX1;
                }
            }
        }
        log::debug!("QSPI transfer_in_place received {:x?}", words);
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

// TODO: Create an async implementation
impl embedded_hal_async::spi::SpiBus<u8> for Qspi {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        embedded_hal::spi::SpiBus::read(self, words)
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        embedded_hal::spi::SpiBus::write(self, words)
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        embedded_hal::spi::SpiBus::transfer(self, read, write)
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        embedded_hal::spi::SpiBus::transfer_in_place(self, words)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_hal::spi::SpiBus::flush(self)
    }
}

// A SPI implementation that uses direct control. Used to create a 400kHz clock.
// Doesn't seem to be needed.
/*
pub struct QspiDirect {}

impl embedded_hal::spi::ErrorType for QspiDirect {
    type Error = SdError;
}

impl QspiDirect {
    pub fn new() -> Self {
        // Enable direct control of SSEL, SCLK, and SDO[0]
        unsafe {
            (*pac::QSPI).DIRECT = (1 << pac::DIRECT_EN_SCLK)   // Enable SCLK control
                                | (1 << pac::DIRECT_EN_SDO) // Enable SDO[0] control
                                | (1 << pac::DIRECT_OP_SDOE) // Enable SDO[0] control
        }

        Self {}
    }
}

impl embedded_hal_async::spi::SpiBus<u8> for QspiDirect {
    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        // 400kHz = 2.5µs per cycle, so 1.25µs per half cycle
        const QUARTER_CYCLE_NANOS: u64 = 625;

        for byte in words.iter_mut() {
            let mut received = 0u8;

            // Transfer one byte, bit by bit
            for bit in (0..8).rev() {
                let start = embassy_time::Instant::now();

                // Set MOSI (SDO[0])
                let bit_out = (*byte >> bit) & 1;
                unsafe {
                    let mut reg = (*pac::QSPI).DIRECT;
                    reg &= !(1 << pac::DIRECT_OP_SDO); // Clear only SDO[0] bit
                    reg |= (bit_out as u32) << pac::DIRECT_OP_SDO; // Set new SDO[0] value
                    (*pac::QSPI).DIRECT = reg;

                    // Spin until quarter cycle (before clock high)
                    while start.elapsed().as_ticks() < QUARTER_CYCLE_NANOS {
                        core::hint::spin_loop();
                    }

                    // Clock high
                    (*pac::QSPI).DIRECT |= 1 << pac::DIRECT_OP_SCLK;

                    // Spin until half cycle + quarter cycle (before sampling)
                    while start.elapsed().as_ticks() < 2 * QUARTER_CYCLE_NANOS {
                        core::hint::spin_loop();
                    }

                    // Sample MISO (SDI[1])
                    let raw_direct = (*pac::QSPI).DIRECT;
                    let sdi = (raw_direct >> (pac::DIRECT_IP_SDI + 1)) & 1;
                    received |= (sdi as u8) << bit;

                    // Spin until full cycle + quarter cycle (before clock low)
                    while start.elapsed().as_ticks() < 3 * QUARTER_CYCLE_NANOS {
                        core::hint::spin_loop();
                    }

                    // Clock low
                    (*pac::QSPI).DIRECT &= !(1 << pac::DIRECT_OP_SCLK);

                    // Spin until end of cycle
                    while start.elapsed().as_ticks() < 4 * QUARTER_CYCLE_NANOS {
                        core::hint::spin_loop();
                    }
                }
            }

            *byte = received;
        }

        Ok(())
    }

    async fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    async fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    async fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        unimplemented!()
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        unimplemented!()
    }
}
*/
