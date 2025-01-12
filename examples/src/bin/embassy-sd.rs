#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use embassy_embedded_hal::{shared_bus::asynch::spi::SpiDeviceWithConfig, SetConfig};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use mpfs_hal::pac;
use mpfs_hal::qspi::{Qspi, SpiConfig, SpiFrequency};
use sdspi::{sd_init, SdSpi};
use static_cell::StaticCell;

static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, Qspi>> = StaticCell::new();

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello");

    let mut spi = Qspi {};
    spi.set_config(&SpiConfig::default()).unwrap();
    let mut cs = SdChipSelect {};

    loop {
        match sd_init(&mut spi, &mut cs).await {
            Ok(_) => break,
            Err(e) => {
                println!("Sd init error: {:?}", e);
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }
    println!("sd_init complete");

    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    let spid = SpiDeviceWithConfig::new(spi_bus, cs, SpiConfig::default());
    let mut sd = SdSpi::<_, _, aligned::A4>::new(spid, embassy_time::Delay);

    while sd.init().await.is_err() {
        println!("Failed to init card, retrying...");
        embassy_time::Timer::after_millis(500).await;
    }
    // Increase the speed up to the SD max of 25mhz
    let mut config = SpiConfig::default();
    config.frequency = SpiFrequency::F25_000_000;
    sd.spi().set_config(config);

    let size = sd.size().await;
    println!("Initialization complete! Got card with size {:?}", size);
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
        pac::MSS_GPIO_init(pac::GPIO0_LO);
        pac::MSS_GPIO_config(
            pac::GPIO0_LO,
            pac::mss_gpio_id_MSS_GPIO_12,
            pac::MSS_GPIO_OUTPUT_MODE,
        );
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
