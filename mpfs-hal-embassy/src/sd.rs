#[cfg(feature = "beaglev-fire")]
pub use beaglev_fire::*;
#[cfg(feature = "beaglev-fire")]
mod beaglev_fire {
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
    use mpfs_hal::{gpio, qspi, Peripheral};

    pub use gpio::{SdChipSelect, SdDetect};

    pub fn init() -> (
        SdChipSelect,
        &'static mut Mutex<CriticalSectionRawMutex, qspi::Qspi>,
    ) {
        gpio::init();
        let qspi_bus = crate::qspi::qspi_bus();
        (SdChipSelect::take().unwrap(), qspi_bus)
    }
}
