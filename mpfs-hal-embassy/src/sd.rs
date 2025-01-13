use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use mpfs_hal::{pac, qspi, Peripheral};

pub fn init() -> (
    SdChipSelect,
    &'static mut Mutex<CriticalSectionRawMutex, qspi::Qspi>,
) {
    unsafe {
        pac::MSS_GPIO_init(pac::GPIO0_LO);
        pac::MSS_GPIO_config(
            pac::GPIO0_LO,
            pac::mss_gpio_id_MSS_GPIO_12,
            pac::MSS_GPIO_OUTPUT_MODE,
        );
    }
    let qspi_bus = super::qspi::qspi_bus();
    (SdChipSelect::take().unwrap(), qspi_bus)
}

static mut SDCS_TAKEN: bool = false;

pub struct SdChipSelect {
    _private: (),
}

impl Peripheral for SdChipSelect {
    fn take() -> Option<Self> {
        critical_section::with(|_| unsafe {
            if SDCS_TAKEN {
                None
            } else {
                SDCS_TAKEN = true;
                Some(Self { _private: () })
            }
        })
    }

    unsafe fn steal() -> Self {
        Self { _private: () }
    }
}

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
