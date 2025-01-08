#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use embedded_hal_async::spi::SpiBus;
use mpfs_hal::pac;
// use sdspi::sd_init;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello");

    let mut spi = Qspi {};
    // let mut cs = SdCs {};
    // loop {
    //     match sd_init(&mut spi, &mut cs).await {
    //         Ok(_) => break,
    //         Err(e) => {
    //             println!("Sd init error: {:?}", e);
    //             embassy_time::Timer::after_millis(10).await;
    //         }
    //     }
    // }
    spi.write(&[0xFF; 10]).await.unwrap();
    println!("Sd init complete");

    // let spi_bus = SPI_BUS.init(Mutex::new(spi));

    // let spid = SpiDeviceWithConfig::new(spi_bus, cs, config);
    // let mut sd = SdSpi::<_, _, aligned::A1>::new(spid, embassy_time::Delay);

    // loop {
    //     // Initialize the card
    //     if sd.init().await.is_ok() {
    //         // Increase the speed up to the SD max of 25mhz

    //         let mut config = Config::default();
    //         config.frequency = 25_000_000;
    //         sd.spi().set_config(config);
    //         defmt::info!("Initialization complete!");

    //         break;
    //     }
    //     defmt::info!("Failed to init card, retrying...");
    //     embassy_time::Delay.delay_ns(5000u32).await;
    // }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}

#[mpfs_hal::init_once]
fn config() {
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
        pac::MSS_QSPI_configure(&pac::mss_qspi_config {
            clk_div: pac::mss_qspi_clk_div_t_MSS_QSPI_CLK_DIV_30,
            sample: pac::MSS_QSPI_SAMPLE_POSAGE_SPICLK as u8, // sample at the rising edge of the SPI clock
            spi_mode: pac::mss_qspi_protocol_mode_t_MSS_QSPI_MODE0, // Idle clock is low
            io_format: pac::mss_qspi_io_format_t_MSS_QSPI_NORMAL,
            xip: pac::MSS_QSPI_DISABLE as u8,
            xip_addr: 0,
        });
    }
    println!("Config complete");
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

impl embedded_hal::spi::SpiBus<u8> for Qspi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
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
                println!("Warning: read not complete");
                if ((*pac::QSPI).STATUS & pac::STTS_FLAGSX4_MASK) != 0 {
                    (*pac::QSPI).RXDATAX4;
                } else {
                    (*pac::QSPI).RXDATAX1;
                }
            }
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
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
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
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
                println!("Warning: read not complete");
                if ((*pac::QSPI).STATUS & pac::STTS_FLAGSX4_MASK) != 0 {
                    (*pac::QSPI).RXDATAX4;
                } else {
                    (*pac::QSPI).RXDATAX1;
                }
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
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
                println!("Warning: read not complete");
                if ((*pac::QSPI).STATUS & pac::STTS_FLAGSX4_MASK) != 0 {
                    (*pac::QSPI).RXDATAX4;
                } else {
                    (*pac::QSPI).RXDATAX1;
                }
            }
        }
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
