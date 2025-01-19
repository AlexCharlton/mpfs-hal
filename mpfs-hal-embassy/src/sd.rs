#[cfg(feature = "beaglev-fire")]
pub use beaglev_fire::*;
#[cfg(feature = "beaglev-fire")]
mod beaglev_fire {
    use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
    use mpfs_hal::{gpio, pac, qspi, Peripheral};

    pub fn init() -> (
        SdChipSelect,
        &'static mut Mutex<CriticalSectionRawMutex, qspi::Qspi>,
    ) {
        unsafe {
            pac::MSS_GPIO_init(pac::GPIO0_LO);
            pac::MSS_GPIO_init(pac::GPIO2_LO);
            SdChipSelect::steal().pin.config_output();
            SdDetect::steal()
                .pin
                .config_input(gpio::InterruptTrigger::default());
        }
        let qspi_bus = crate::qspi::qspi_bus();
        (SdChipSelect::take().unwrap(), qspi_bus)
    }

    //---------------------------------------------------------------------------

    static mut SDCS_TAKEN: bool = false;

    pub struct SdChipSelect {
        pin: gpio::Pin,
    }

    impl Peripheral for SdChipSelect {
        fn take() -> Option<Self> {
            critical_section::with(|_| unsafe {
                if SDCS_TAKEN {
                    None
                } else {
                    SDCS_TAKEN = true;
                    Some(Self {
                        pin: gpio::Pin::new(
                            12,
                            gpio::GpioPeripheral::Mss(pac::GPIO0_LO),
                            1000, // Never used
                        ),
                    })
                }
            })
        }

        unsafe fn steal() -> Self {
            Self {
                pin: gpio::Pin::new(
                    12,
                    gpio::GpioPeripheral::Mss(pac::GPIO0_LO),
                    1000, // Never used
                ),
            }
        }
    }

    impl embedded_hal::digital::OutputPin for SdChipSelect {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.pin.set_low();
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

    //---------------------------------------------------------------------------

    use embedded_hal::digital::InputPin;
    use embedded_hal_async::digital::Wait;

    static mut SD_DETECT_TAKEN: bool = false;

    pub struct SdDetect {
        pin: gpio::Pin,
    }

    impl Peripheral for SdDetect {
        fn take() -> Option<Self> {
            critical_section::with(|_| unsafe {
                if SD_DETECT_TAKEN {
                    None
                } else {
                    SD_DETECT_TAKEN = true;
                    Some(Self {
                        pin: gpio::Pin::new(
                            31,
                            gpio::GpioPeripheral::Mss(pac::GPIO2_LO),
                            gpio::SD_DETECT_INTERRUPT_IDX,
                        ),
                    })
                }
            })
        }

        unsafe fn steal() -> Self {
            Self {
                pin: gpio::Pin::new(
                    31,
                    gpio::GpioPeripheral::Mss(pac::GPIO2_LO),
                    gpio::SD_DETECT_INTERRUPT_IDX,
                ),
            }
        }
    }

    impl embedded_hal::digital::ErrorType for SdDetect {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal::digital::InputPin for SdDetect {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(self.pin.is_high())
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.pin.is_high())
        }
    }

    impl embedded_hal_async::digital::Wait for SdDetect {
        async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
            if !self.pin.is_high() {
                gpio::InputFuture::new(self.pin, gpio::InterruptTrigger::LevelHigh).await
            } else {
                Ok(())
            }
        }

        async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
            if self.pin.is_high() {
                gpio::InputFuture::new(self.pin, gpio::InterruptTrigger::LevelLow).await
            } else {
                Ok(())
            }
        }

        async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
            gpio::InputFuture::new(self.pin, gpio::InterruptTrigger::EdgePositive).await
        }

        async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
            gpio::InputFuture::new(self.pin, gpio::InterruptTrigger::EdgeNegative).await
        }

        async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
            gpio::InputFuture::new(self.pin, gpio::InterruptTrigger::EdgeBoth).await
        }
    }

    impl SdDetect {
        pub fn is_inserted(&mut self) -> bool {
            self.is_low().unwrap()
        }

        pub async fn wait_for_inserted(
            &mut self,
        ) -> Result<(), <Self as embedded_hal::digital::ErrorType>::Error> {
            self.wait_for_low().await
        }

        pub async fn wait_for_removed(
            &mut self,
        ) -> Result<(), <Self as embedded_hal::digital::ErrorType>::Error> {
            self.wait_for_high().await
        }
    }
}
