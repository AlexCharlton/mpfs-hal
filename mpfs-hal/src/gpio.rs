use crate::pac;

#[derive(Debug)]
#[allow(dead_code)]
enum GpioPeripheral {
    Mss(*mut pac::GPIO_TypeDef),
    FpgaCore(usize),
}

#[derive(Debug)]
pub struct Pin {
    number: u32,
    peripheral: GpioPeripheral,
}

impl Pin {
    fn config_output(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    pac::MSS_GPIO_config(typedef, self.number, pac::MSS_GPIO_OUTPUT_MODE);
                }
                GpioPeripheral::FpgaCore(address) => {
                    // TODO: Implement FPGA Core GPIO configuration
                    unimplemented!()
                }
            }
        }
    }

    fn set_high(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    pac::MSS_GPIO_set_output(typedef, self.number, 1);
                }
                GpioPeripheral::FpgaCore(address) => {
                    // TODO: Implement FPGA Core GPIO configuration
                    unimplemented!()
                }
            }
        }
    }

    fn set_low(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    pac::MSS_GPIO_set_output(typedef, self.number, 0);
                }
                GpioPeripheral::FpgaCore(address) => {
                    // TODO: Implement FPGA Core GPIO configuration
                    unimplemented!()
                }
            }
        }
    }
}

pub trait GpioPin {
    fn address(&self) -> Pin;
}

pub struct Output {
    pub pin: Pin,
}

impl Output {
    pub fn new<P: GpioPin>(pin: P) -> Self {
        let address = pin.address();
        address.config_output();
        Self { pin: address }
    }
}

impl embedded_hal::digital::ErrorType for Output {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for Output {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }
}

macro_rules! impl_mss_gpio_pin {
    ($pin:ident, $n:expr, $peripheral:expr) => {
        paste! {
            impl_mss_gpio_pin!($pin, [<$pin _TAKEN>], $n, $peripheral);
        }
    };

    ($PIN:ident, $PIN_TAKEN:ident, $num:expr, $peripheral:expr) => {
        pub struct $PIN {
            _private: (),
        }
        static mut $PIN_TAKEN: bool = false;

        impl crate::Peripheral for $PIN {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $PIN_TAKEN {
                        None
                    } else {
                        $PIN_TAKEN = true;
                        Some(Self { _private: () })
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }

        impl GpioPin for $PIN {
            fn address(&self) -> Pin {
                Pin {
                    number: $num,
                    peripheral: $peripheral,
                }
            }
        }
    };
}

#[cfg(feature = "beaglev-fire")]
pub use beaglev_fire::*;
#[cfg(feature = "beaglev-fire")]
mod beaglev_fire {
    use super::*;
    use crate::pac;
    use paste::paste;

    // Move to PAC?
    static mut P8_CORE_GPIO: pac::gpio_instance_t = pac::gpio_instance_t {
        base_addr: 0x41100000,
        apb_bus_width: pac::__gpio_apb_width_t_GPIO_APB_32_BITS_BUS,
    };

    static mut P9_CORE_GPIO: pac::gpio_instance_t = pac::gpio_instance_t {
        base_addr: 0x41200000,
        apb_bus_width: pac::__gpio_apb_width_t_GPIO_APB_32_BITS_BUS,
    };

    pub fn init() {
        unsafe {
            pac::MSS_GPIO_init(pac::GPIO2_LO);
            pac::mss_enable_fabric();
            let mut p8: pac::gpio_instance_t = P8_CORE_GPIO;
            pac::GPIO_init(&mut p8, P8_CORE_GPIO.base_addr, P8_CORE_GPIO.apb_bus_width);
            let mut p9: pac::gpio_instance_t = P9_CORE_GPIO;
            pac::GPIO_init(&mut p9, P9_CORE_GPIO.base_addr, P9_CORE_GPIO.apb_bus_width);

            /*
            // MOVE
            pac::GPIO_config(
                &mut p8,
                pac::mss_gpio_id_MSS_GPIO_0,
                pac::MSS_GPIO_OUTPUT_MODE,
            );

            let mut gpio_outputs = pac::GPIO_get_outputs(&mut p8);
            // Set 0 and 4 to high
            gpio_outputs |= pac::MSS_GPIO_0_MASK | pac::MSS_GPIO_4_MASK;
            pac::GPIO_set_outputs(&mut p8, gpio_outputs);
            */
        }
    }

    // From the default Cape documentation: https://git.beagleboard.org/beaglev-fire/gateware/-/blob/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT/Readme.md
    /*
    | P8_3   | MSS GPIO_2[0]              |   53  | User LED 0  |
    | P8_4   | MSS GPIO_2[1]              |   53  | User LED 1  |
    | P8_5   | MSS GPIO_2[2]              |   53  | User LED 2  |
    | P8_6   | MSS GPIO_2[3]              |   53  | User LED 3  |
    | P8_7   | MSS GPIO_2[4]              |   53  | User LED 4  |
    | P8_8   | MSS GPIO_2[5]              |   53  | User LED 5  |
    | P8_9   | MSS GPIO_2[6]              |   53  | User LED 6  |
    | P8_10  | MSS GPIO_2[7]              |   53  | User LED 7  |
    | P8_11  | MSS GPIO_2[8]              |   53  | User LED 8  |
    | P8_12  | MSS GPIO_2[9]              |   53  | User LED 9  |
    | P8_14  | MSS GPIO_2[11]             |   53  | User LED 11 |
    | P8_15  | MSS GPIO_2[12]             |   53  | GPIO        |
    | P8_16  | MSS GPIO_2[13]             |   53  | GPIO        |
    | P8_17  | MSS GPIO_2[14]             |   53  | GPIO        |
    | P8_18  | MSS GPIO_2[15]             |   53  | GPIO        |
    | P8_20  | MSS GPIO_2[17]             |   53  | GPIO        |
    | P8_21  | MSS GPIO_2[18]             |   53  | GPIO        |
    | P8_22  | MSS GPIO_2[19]             |   53  | GPIO        |
    | P8_23  | MSS GPIO_2[20]             |   53  | GPIO        |
    | P8_24  | MSS GPIO_2[21]             |   53  | GPIO        |
    | P8_25  | MSS GPIO_2[22]             |   53  | GPIO        |
    | P8_26  | MSS GPIO_2[23]             |   53  | GPIO        |
    | P8_27  | MSS GPIO_2[24]             |   53  | GPIO        |
    | P8_28  | MSS GPIO_2[25]             |   53  | GPIO        |
    | P8_29  | MSS GPIO_2[26]             |   53  | GPIO        |
    | P8_30  | MSS GPIO_2[27]             |   53  | GPIO        |
    | P8_31  | core_gpio[0] @ 0x41100000  |  126  | GPIO        |
    | P8_32  | core_gpio[1] @ 0x41100000  |  127  | GPIO        |
    | P8_33  | core_gpio[2] @ 0x41100000  |  128  | GPIO        |
    | P8_34  | core_gpio[3] @ 0x41100000  |  129  | GPIO        |
    | P8_35  | core_gpio[4] @ 0x41100000  |  130  | GPIO        |
    | P8_36  | core_gpio[5] @ 0x41100000  |  131  | GPIO        |
    | P8_37  | core_gpio[6] @ 0x41100000  |  132  | GPIO        |
    | P8_38  | core_gpio[7] @ 0x41100000  |  133  | GPIO        |
    | P8_39  | core_gpio[8] @ 0x41100000  |  134  | GPIO        |
    | P8_40  | core_gpio[9] @ 0x41100000  |  135  | GPIO        |
    | P8_41  | core_gpio[10] @ 0x41100000 |  136  | GPIO        |
    | P8_42  | core_gpio[11] @ 0x41100000 |  137  | GPIO        |
    | P8_43  | core_gpio[12] @ 0x41100000 |  138  | GPIO        |
    | P8_44  | core_gpio[13] @ 0x41100000 |  139  | GPIO        |
    | P8_45  | core_gpio[14] @ 0x41100000 |  140  | GPIO        |
    | P8_46  | core_gpio[15] @ 0x41100000 |  141  | GPIO        |
    | P9_12  | core_gpio[1] @ 0x41200000  |  143  | GPIO        |
    | P9_15  | core_gpio[4] @ 0x41200000  |  146  | GPIO        |
    | P9_23  | core_gpio[10] @ 0x41200000 |  152  | GPIO        |
    | P9_25  | core_gpio[12] @ 0x41200000 |  154  | GPIO        |
    | P9_27  | core_gpio[14] @ 0x41200000 |  156  | GPIO        |
    | P9_30  | core_gpio[17] @ 0x41200000 |  159  | GPIO        |
    | P9_41  | core_gpio[19] @ 0x41200000 |  161  | GPIO        |
    */
    impl_mss_gpio_pin!(P8_3, 0, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_4, 1, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_5, 2, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_6, 3, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_7, 4, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_8, 5, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_9, 6, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_10, 7, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_11, 8, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_12, 9, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_14, 11, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_15, 12, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_16, 13, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_17, 14, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_18, 15, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_20, 17, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_21, 18, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_22, 19, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_23, 20, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_24, 21, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_25, 22, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_26, 23, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_27, 24, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_28, 25, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_29, 26, GpioPeripheral::Mss(pac::GPIO2_LO));
    impl_mss_gpio_pin!(P8_30, 27, GpioPeripheral::Mss(pac::GPIO2_LO));
}
