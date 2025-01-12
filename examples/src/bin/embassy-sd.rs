#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use embassy_embedded_hal::{shared_bus::asynch::spi::SpiDeviceWithConfig, SetConfig};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_hal::spi::{Phase, Polarity};
use mpfs_hal::pac;
use sdspi::{sd_init, SdSpi};
use static_cell::StaticCell;

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, Qspi>> = StaticCell::new();

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello");

    let mut spi = Qspi {};
    spi.set_config(&SpiConfig::default()).unwrap();
    let mut cs = SdChipSelect {};

    cs.set_low().unwrap();
    Timer::after(Duration::from_millis(100)).await;
    cs.set_high().unwrap();
    println!("first cs");

    let buffer = aligned::Aligned::<aligned::A4, _>([
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    ]);
    spi.write(buffer.as_slice()).unwrap();
    println!("Initial write");

    let mut buffer = aligned::Aligned::<aligned::A4, _>([0x40, 0, 0, 0, 0, 0x95]); // Reset
    cs.set_low().unwrap();
    spi.write(buffer.as_mut_slice()).unwrap();
    let mut buffer = [0xFF];
    loop {
        spi.read(&mut buffer).unwrap();
        if buffer[0] != 0xFF {
            break;
        }
    }
    cs.set_high().unwrap();
    if buffer[0] == 0x01 {
        println!("Card is present");
    } else {
        println!("Card is not present");
    }

    println!("result: {:x?}", buffer);

    let mut buffer = aligned::Aligned::<aligned::A4, _>([0x48, 0x00, 0x00, 0x01, 0xAA, 0x87]); // CMD8
    cs.set_low().unwrap();
    spi.write(buffer.as_mut_slice()).unwrap();
    let mut buffer = [0xFF];
    loop {
        spi.read(&mut buffer).unwrap();
        if buffer[0] != 0xFF {
            break;
        }
    }
    // println!("result: {:x?}", buffer);
    let mut buffer = [0xFF; 4];
    spi.read(&mut buffer).unwrap();
    cs.set_high().unwrap();
    println!("result: {:x?}", buffer);

    // loop {
    //     match sd_init(&mut spi, &mut cs).await {
    //         Ok(_) => break,
    //         Err(e) => {
    //             println!("Sd init error: {:?}", e);
    //             embassy_time::Timer::after_millis(10).await;
    //         }
    //     }
    // }
    // println!("sd_init complete");

    // let spi_bus = SPI_BUS.init(Mutex::new(spi));
    // let spid = SpiDeviceWithConfig::new(spi_bus, cs, SpiConfig::default());
    // let mut sd = SdSpi::<_, _, aligned::A1>::new(spid, embassy_time::Delay);

    // while sd.init().await.is_err() {
    //     println!("Failed to init card, retrying...");
    //     embassy_time::Timer::after_millis(500).await;
    // }
    // // Increase the speed up to the SD max of 25mhz
    // let mut config = SpiConfig::default();
    // config.frequency = SpiFrequency::F25_000_000;
    // sd.spi().set_config(config);

    println!("Initialization complete!");
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Info);
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
        pac::MSS_GPIO_init(pac::GPIO0_LO);
        pac::MSS_GPIO_config(
            pac::GPIO0_LO,
            pac::mss_gpio_id_MSS_GPIO_12,
            pac::MSS_GPIO_OUTPUT_MODE,
        );
        // pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_QSPI_INT_OFFSET, 2);
        // pac::PLIC_EnableIRQ(pac::PLIC_IRQn_Type_PLIC_QSPI_INT_OFFSET);

        pac::MSS_QSPI_init();
    }
    log::info!("Config complete");
}

struct SdChipSelect {}

impl embedded_hal::digital::OutputPin for SdChipSelect {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe {
            pac::MSS_GPIO_set_output(pac::GPIO0_LO, pac::mss_gpio_id_MSS_GPIO_12, 0);
        }
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe {
            pac::MSS_GPIO_set_output(pac::GPIO0_LO, pac::mss_gpio_id_MSS_GPIO_12, 1);
        }
        Ok(())
    }
}

impl embedded_hal::digital::ErrorType for SdChipSelect {
    type Error = core::convert::Infallible;
}

struct Qspi {}

impl embedded_hal::spi::ErrorType for Qspi {
    type Error = SdError;
}

#[derive(Debug)]
struct SdError {}

impl embedded_hal::spi::Error for SdError {
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
            frequency: SpiFrequency::F3_333_333,
            phase: Phase::CaptureOnFirstTransition,
            polarity: Polarity::IdleLow,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum SpiFrequency {
    F50_000_000 = 1,  // 100MHz / 2
    F25_000_000 = 2,  // 100MHz / 4
    F16_666_666 = 3,  // 100MHz / 6
    F12_500_000 = 4,  // 100MHz / 8
    F10_000_000 = 5,  // 100MHz / 10
    F8_333_333 = 6,   // 100MHz / 12
    F7_142_857 = 7,   // 100MHz / 14
    F6_250_000 = 8,   // 100MHz / 16
    F5_555_555 = 9,   // 100MHz / 18
    F5_000_000 = 0xA, // 100MHz / 20
    F4_545_454 = 0xB, // 100MHz / 22
    F4_166_666 = 0xC, // 100MHz / 24
    F3_846_153 = 0xD, // 100MHz / 26
    F3_571_428 = 0xE, // 100MHz / 28
    F3_333_333 = 0xF, // 100MHz / 30
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
            log::debug!("QSPI Control Register: {:#032b}", value);
            (*pac::QSPI).CONTROL = value;
        }
        Ok(())
    }
}

impl embedded_hal::spi::SpiBus<u8> for Qspi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        log::debug!("read");
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

            // Enable 32-bit transfer mode
            (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

            // Transfer 32-bit aligned words
            let words_32 = words.as_ptr() as *mut u32;
            let word_count = total_bytes / 4;

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

            // Without this delay the TXDATAX1 FIFO does not get updated with proper data
            pac::sleep_ms(10);

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
        log::debug!("read {:x?}", words);
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        log::debug!("write {:x?}", words);
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

            // Enable 32-bit transfer mode
            (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

            // Transfer 32-bit aligned words
            let words_32 = words.as_ptr() as *const u32;
            let word_count = total_bytes / 4;

            for i in 0..word_count {
                // Wait until transmit FIFO is not full
                while ((*pac::QSPI).STATUS & pac::STTS_TFFULL_MASK) != 0 {
                    core::hint::spin_loop();
                }
                (*pac::QSPI).TXDATAX4 = *words_32.add(i);
            }

            // Disable 32-bit transfer mode
            (*pac::QSPI).CONTROL &= !pac::CTRL_FLAGSX4_MASK;

            // Without this delay the TXDATAX1 FIFO does not get updated with proper data
            pac::sleep_ms(10);

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
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        log::debug!("transfer {:x?}", write);
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

            frame_ctrl |= pac::FRMS_FWORD_MASK; // Set full word mode
            (*pac::QSPI).FRAMES = frame_ctrl;

            // Enable 32-bit transfer mode
            (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

            // Transfer 32-bit aligned words
            let write_32 = write.as_ptr() as *const u32;
            let read_32 = read.as_ptr() as *mut u32;
            let word_count = total_bytes / 4;
            let tx_word_count = write.len() / 4;
            let rx_word_count = read.len() / 4;

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

            // Without this delay the TXDATAX1 FIFO does not get updated with proper data
            pac::sleep_ms(10);

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
        log::debug!("transfer received {:x?}", read);
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        log::debug!("transfer_in_place {:x?}", words);
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

            // Enable 32-bit transfer mode
            (*pac::QSPI).CONTROL |= pac::CTRL_FLAGSX4_MASK;

            // Transfer 32-bit aligned words
            let words_32 = words.as_ptr() as *mut u32;
            let word_count = total_bytes / 4;

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

            // Without this delay the TXDATAX1 FIFO does not get updated with proper data
            pac::sleep_ms(10);

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
        log::debug!("transfer_in_place received {:x?}", words);
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
