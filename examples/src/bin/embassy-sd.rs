#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use alloc::vec;
use mpfs_hal::pac;
use sdspi::sd_init;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello");

    let mut spi = SdSpi {};
    let mut cs = SdCs {};
    loop {
        match sd_init(&mut spi, &mut cs).await {
            Ok(_) => break,
            Err(e) => {
                println!("Sd init error: {:?}", e);
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }
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

struct SdCs {}

impl embedded_hal::digital::OutputPin for SdCs {
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

impl embedded_hal::digital::ErrorType for SdCs {
    type Error = core::convert::Infallible;
}

struct SdSpi {}

impl embedded_hal::spi::ErrorType for SdSpi {
    type Error = SdError;
}

#[derive(Debug)]
struct SdError {}

impl embedded_hal::spi::Error for SdError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

impl embedded_hal_async::spi::SpiBus<u8> for SdSpi {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.transfer(words, &[]).await?;
        Ok(())
    }
    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.transfer(&mut [], words).await?;
        Ok(())
    }

    // TODO create a decent implementation of this
    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        // println!("transfering: {:?}", write);
        unsafe {
            pac::MSS_QSPI_polled_transfer_block(
                3,
                write.as_ptr() as *const core::ffi::c_void,
                (write.len() as u32).saturating_sub(3 + 1), // Address bytes + 1 byte for the command
                read.as_mut_ptr() as *mut core::ffi::c_void,
                read.len() as u32,
                0,
            );
        }
        println!("read from transfer: {:?}", read);
        Ok(())
    }
    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut read = vec![0u8; words.len()];
        self.transfer(&mut read, words).await?;
        words.copy_from_slice(&read[..words.len()]);
        Ok(())
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
