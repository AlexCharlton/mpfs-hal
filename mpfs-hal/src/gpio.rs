//! The MPFS broadly uses 2 GPIO interfaces: the MSS GPIO interface and the FPGA Core GPIO interface.
//!
//! On the BeagleV-Fire, P8_3 through P9_30 use the MSS GPIO interface. The remainder use the FPGA Core GPIO interface. Both interfaces are routed through the FPGA and thus depend on the FPGA configuration to work correctly. See the [Default cape gateware](https://git.beagleboard.org/beaglev-fire/gateware/-/tree/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT) for the expected configuration. Note that if you want to use interrupts for the Core GPIO interface, "fixed config" needs to be disabled, or the interrupts need to be otherwise configured in the gateware. The [CoreGPIO_LCD.tcl](https://git.beagleboard.org/beaglev-fire/gateware/-/blob/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT/CoreGPIO_LCD.tcl?ref_type=heads#L7) file controls this config for the P8 pins, while [CoreGPIO_P9.tcl](https://git.beagleboard.org/beaglev-fire/gateware/-/blob/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT/CoreGPIO_P9.tcl?ref_type=heads)

extern crate alloc;

use core::convert::Infallible;
use core::future::Future;
use core::task::{Context, Poll, Waker};

use crate::pac;

#[derive(Default)]
struct GpioInterrupt {
    waker: Option<Waker>,
    triggered: bool,
}

const NUM_INTERRUPTS: usize = 64;
static mut GPIO_INTERRUPTS: [GpioInterrupt; NUM_INTERRUPTS] = [const {
    GpioInterrupt {
        waker: None,
        triggered: false,
    }
}; NUM_INTERRUPTS];

#[repr(u8)]
#[derive(Default, Copy, Clone, Debug)]
#[doc(hidden)]
pub enum InterruptTrigger {
    #[default]
    LevelHigh = pac::GPIO_IRQ_LEVEL_HIGH as u8,
    LevelLow = pac::GPIO_IRQ_LEVEL_LOW as u8,
    EdgePositive = pac::GPIO_IRQ_EDGE_POSITIVE as u8,
    EdgeNegative = pac::GPIO_IRQ_EDGE_NEGATIVE as u8,
    EdgeBoth = pac::GPIO_IRQ_EDGE_BOTH as u8,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
#[doc(hidden)]
pub enum GpioPeripheral {
    Mss(*mut pac::GPIO_TypeDef),
    FpgaCore(pac::gpio_instance_t),
}

#[doc(hidden)]
#[derive(Debug, Clone, Copy)]
pub struct Pin {
    number: u8,
    peripheral: GpioPeripheral,
    interrupt_idx: u8,
    plic_idx: u8,
}

impl Pin {
    // Meant to be used only by board-support code
    #[doc(hidden)]
    pub fn new(number: u8, peripheral: GpioPeripheral, interrupt_idx: u8, plic_idx: u8) -> Self {
        Self {
            number,
            peripheral,
            interrupt_idx,
            plic_idx,
        }
    }

    #[doc(hidden)]
    pub fn config_output(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    log::trace!("Configuring MSS GPIO {:?} to output", self);
                    pac::MSS_GPIO_config(typedef, self.number as u32, pac::MSS_GPIO_OUTPUT_MODE);
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    log::trace!("Configuring FPGA Core GPIO {:?} to output", self);
                    pac::GPIO_config(&mut address, self.number as u32, pac::MSS_GPIO_OUTPUT_MODE);
                }
            }
        }
    }

    #[doc(hidden)]
    pub fn config_input(&self, config: InterruptTrigger) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    log::trace!(
                        "Configuring MSS GPIO {:?} to input with trigger {:?}",
                        self,
                        config
                    );
                    pac::MSS_GPIO_config(
                        typedef,
                        self.number as u32,
                        pac::MSS_GPIO_INPUT_MODE | config as u32,
                    );
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    log::trace!(
                        "Configuring FPGA Core GPIO {:?} to input with trigger {:?}",
                        self,
                        config
                    );
                    pac::GPIO_config(
                        &mut address,
                        self.number as u32,
                        pac::MSS_GPIO_INPUT_MODE | config as u32,
                    );
                }
            }
        }
    }

    #[doc(hidden)]
    pub fn set_high(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    log::trace!("Setting MSS GPIO {:?} to high", self);
                    pac::MSS_GPIO_set_output(typedef, self.number as u32, 1);
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    let mut gpio_outputs = pac::GPIO_get_outputs(&mut address);
                    gpio_outputs |= 1 << self.number;
                    log::trace!(
                        "Setting FPGA Core GPIO {:?} to high with value {:?}",
                        self,
                        gpio_outputs
                    );
                    pac::GPIO_set_outputs(&mut address, gpio_outputs);
                }
            }
        }
    }

    #[doc(hidden)]
    pub fn set_low(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    log::trace!("Setting MSS GPIO {:?} to low", self);
                    pac::MSS_GPIO_set_output(typedef, self.number as u32, 0);
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    let mut gpio_outputs = pac::GPIO_get_outputs(&mut address);
                    gpio_outputs &= !(1 << self.number);
                    log::trace!(
                        "Setting FPGA Core GPIO {:?} to low with value {:?}",
                        self,
                        gpio_outputs
                    );
                    pac::GPIO_set_outputs(&mut address, gpio_outputs);
                }
            }
        }
    }

    #[doc(hidden)]
    pub fn is_high(&self) -> bool {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    let inputs = (*typedef).GPIO_IN;
                    inputs & (1 << self.number) != 0
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    let inputs = pac::GPIO_get_inputs(&mut address);
                    inputs & (1 << self.number) != 0
                }
            }
        }
    }

    fn enable_interrupt(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    pac::MSS_GPIO_enable_irq(typedef, self.number as u32);
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    pac::GPIO_enable_irq(&mut address, self.number as u32);
                    pac::PLIC_EnableIRQ(self.plic_idx as u32);
                }
            }
        }
    }

    fn clear_interrupt(peripheral: GpioPeripheral, number: u8) {
        unsafe {
            match peripheral {
                GpioPeripheral::Mss(addr) => {
                    pac::MSS_GPIO_clear_irq(addr, number as u32);
                }
                GpioPeripheral::FpgaCore(addr) => {
                    let mut address = addr;
                    pac::GPIO_clear_irq(&mut address, number as u32);
                }
            }
        }
    }

    fn disable_interrupt(peripheral: GpioPeripheral, number: u8) {
        unsafe {
            match peripheral {
                GpioPeripheral::Mss(addr) => {
                    pac::MSS_GPIO_disable_irq(addr, number as u32);
                }
                GpioPeripheral::FpgaCore(addr) => {
                    let mut address = addr;
                    pac::GPIO_disable_irq(&mut address, number as u32);
                }
            }
        }
    }

    unsafe fn triggered(&self) -> bool {
        GPIO_INTERRUPTS[self.interrupt_idx as usize].triggered
    }

    unsafe fn set_waker(&mut self, waker: Waker) {
        GPIO_INTERRUPTS[self.interrupt_idx as usize].waker = Some(waker);
    }
}

/// All GPIO pins implement this trait
pub trait GpioPin: 'static {
    #[doc(hidden)]
    fn address(&self) -> Pin;
}

pub struct Output {
    pub pin: Pin,
}

//----------------------------------------------------------------------
// Output

impl Output {
    pub fn new<P: GpioPin>(pin: P) -> Self {
        let address = pin.address();
        address.config_output();
        Self { pin: address }
    }
}

impl embedded_hal::digital::ErrorType for Output {
    type Error = Infallible;
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

//----------------------------------------------------------------------
// Input

pub struct Input {
    pub pin: Pin,
}

impl Input {
    pub fn new<P: GpioPin>(pin: P) -> Self {
        let address = pin.address();
        address.config_input(InterruptTrigger::default());
        Self { pin: address }
    }
}

impl embedded_hal::digital::ErrorType for Input {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::InputPin for Input {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.pin.is_high())
    }
}

//----------------------------------------------------------------------
// Interrupt

#[doc(hidden)]
pub struct InputFuture {
    pin: Pin,
}

impl InputFuture {
    pub fn new(pin: Pin, interrupt_trigger: InterruptTrigger) -> Self {
        critical_section::with(|_| unsafe {
            GPIO_INTERRUPTS[pin.interrupt_idx as usize].triggered = false;
        });
        pin.config_input(interrupt_trigger);
        pin.enable_interrupt();
        Self { pin }
    }
}

impl Future for InputFuture {
    type Output = Result<(), Infallible>;

    fn poll(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let trigger = critical_section::with(|_| unsafe {
            self.as_mut().pin.set_waker(cx.waker().clone());
            self.as_mut().pin.triggered()
        });

        if trigger {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

impl embedded_hal_async::digital::Wait for Input {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        if !self.pin.is_high() {
            InputFuture::new(self.pin, InterruptTrigger::LevelHigh).await
        } else {
            Ok(())
        }
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        if self.pin.is_high() {
            InputFuture::new(self.pin, InterruptTrigger::LevelLow).await
        } else {
            Ok(())
        }
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self.pin, InterruptTrigger::EdgePositive).await
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self.pin, InterruptTrigger::EdgeNegative).await
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        InputFuture::new(self.pin, InterruptTrigger::EdgeBoth).await
    }
}

//----------------------------------------------------------------------

macro_rules! impl_gpio_pin {
    ($pin:ident, $n:expr, $peripheral:expr, $interrupt_idx:expr, $interrupt_handler:ident, $plic_idx:expr) => {
        paste! {
            impl_gpio_pin!($pin, [<$pin _TAKEN>], [<$pin _INTERRUPT>], $n, $peripheral, $interrupt_idx, $interrupt_handler, $plic_idx);
        }
    };

    ($pin:ident, $n:expr, $peripheral:expr, $interrupt_idx:expr, $interrupt_handler:ident) => {
        paste! {
            impl_gpio_pin!($pin, [<$pin _TAKEN>], [<$pin _INTERRUPT>], $n, $peripheral, $interrupt_idx, $interrupt_handler, 255);
        }
    };

    ($PIN:ident, $PIN_TAKEN:ident, $PIN_INTERRUPT:ident, $num:expr, $peripheral:expr, $interrupt_idx:expr, $interrupt_handler:ident, $plic_idx:expr) => {
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
                    interrupt_idx: $interrupt_idx,
                    plic_idx: $plic_idx as u8,
                }
            }
        }

        #[no_mangle]
        extern "C" fn $interrupt_handler() -> u8 {
            let interrupt = unsafe { &mut GPIO_INTERRUPTS[$interrupt_idx] };
            if let Some(waker) = interrupt.waker.take() {
                interrupt.triggered = true;
                waker.wake();
            }
            Pin::clear_interrupt($peripheral, $num);
            Pin::disable_interrupt($peripheral, $num);
            pac::EXT_IRQ_DISABLE as u8
        }
    };
}

//----------------------------------------------------------------------

#[cfg(feature = "beaglev-fire")]
pub use beaglev_fire::*;
#[cfg(feature = "beaglev-fire")]
mod beaglev_fire {
    // TODO: configure based on what cape is being applied
    // Can we provide some user-customization?
    // https://docs.beagle.cc/boards/beaglev/fire/04-expansion.html
    use super::*;
    use crate::pac;
    use paste::paste;

    const GPIO2_PIN_COUNT: usize = 32;

    // Move to PAC?
    const P8_CORE_GPIO: pac::gpio_instance_t = pac::gpio_instance_t {
        base_addr: 0x41100000,
        apb_bus_width: pac::__gpio_apb_width_t_GPIO_APB_32_BITS_BUS,
    };

    const P9_CORE_GPIO: pac::gpio_instance_t = pac::gpio_instance_t {
        base_addr: 0x41200000,
        apb_bus_width: pac::__gpio_apb_width_t_GPIO_APB_32_BITS_BUS,
    };

    /// To be called before using any GPIO pins
    pub fn init() {
        unsafe {
            // GPIO2 initialization
            pac::MSS_GPIO_init(pac::GPIO2_LO);
            // Route GPIO2 interrupts to the PLIC (Rather than GPIO0 and GPIO1)
            (*pac::SYSREG).GPIO_INTERRUPT_FAB_CR = 0xFFFFFFFF;
            for i in 0..GPIO2_PIN_COUNT {
                pac::PLIC_SetPriority(
                    pac::PLIC_IRQn_Type_PLIC_GPIO0_BIT0_or_GPIO2_BIT0_INT_OFFSET + i as u32,
                    2,
                );
            }

            // FPGA Core GPIO initialization
            pac::mss_enable_fabric();
            let mut p8: pac::gpio_instance_t = P8_CORE_GPIO;
            log::trace!("Initializing FPGA Core GPIO {:?}", p8);
            pac::GPIO_init(&mut p8, P8_CORE_GPIO.base_addr, P8_CORE_GPIO.apb_bus_width);
            let mut p9: pac::gpio_instance_t = P9_CORE_GPIO;
            log::trace!("Initializing FPGA Core GPIO {:?}", p9);
            pac::GPIO_init(&mut p9, P9_CORE_GPIO.base_addr, P9_CORE_GPIO.apb_bus_width);

            for i in pac::PLIC_IRQn_Type_PLIC_F2M_8_INT_OFFSET
                ..=pac::PLIC_IRQn_Type_PLIC_F2M_43_INT_OFFSET
            {
                pac::PLIC_SetPriority(i, 2);
            }
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
    | P8_13  | MSS GPIO_2[10]             |   53  | User LED 10 |
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
    impl_gpio_pin!(
        P8_3,
        0,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        0,
        // The first 14 GPIO2 handlers are misnamed
        // This one handles GPIO2[0]
        PLIC_gpio0_bit0_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_4,
        1,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        1,
        PLIC_gpio0_bit1_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_5,
        2,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        2,
        PLIC_gpio0_bit2_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_6,
        3,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        3,
        PLIC_gpio0_bit3_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_7,
        4,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        4,
        PLIC_gpio0_bit4_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_8,
        5,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        5,
        PLIC_gpio0_bit5_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_9,
        6,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        6,
        PLIC_gpio0_bit6_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_10,
        7,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        7,
        PLIC_gpio0_bit7_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_11,
        8,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        8,
        PLIC_gpio0_bit8_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_12,
        9,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        9,
        PLIC_gpio0_bit9_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_13,
        10,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        10,
        PLIC_gpio0_bit10_or_gpio2_bit13_IRQHandler
    );

    impl_gpio_pin!(
        P8_14,
        11,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        10,
        PLIC_gpio0_bit11_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_15,
        12,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        11,
        PLIC_gpio0_bit12_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_16,
        13,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        12,
        PLIC_gpio0_bit13_or_gpio2_bit13_IRQHandler
    );
    impl_gpio_pin!(
        P8_17,
        14,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        13,
        PLIC_gpio1_bit0_or_gpio2_bit14_IRQHandler
    );
    impl_gpio_pin!(
        P8_18,
        15,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        14,
        PLIC_gpio1_bit1_or_gpio2_bit15_IRQHandler
    );
    impl_gpio_pin!(
        P8_20,
        17,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        15,
        PLIC_gpio1_bit3_or_gpio2_bit17_IRQHandler
    );
    impl_gpio_pin!(
        P8_21,
        18,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        16,
        PLIC_gpio1_bit4_or_gpio2_bit18_IRQHandler
    );
    impl_gpio_pin!(
        P8_22,
        19,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        17,
        PLIC_gpio1_bit5_or_gpio2_bit19_IRQHandler
    );
    impl_gpio_pin!(
        P8_23,
        20,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        18,
        PLIC_gpio1_bit6_or_gpio2_bit20_IRQHandler
    );
    impl_gpio_pin!(
        P8_24,
        21,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        19,
        PLIC_gpio1_bit7_or_gpio2_bit21_IRQHandler
    );
    impl_gpio_pin!(
        P8_25,
        22,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        20,
        PLIC_gpio1_bit8_or_gpio2_bit22_IRQHandler
    );
    impl_gpio_pin!(
        P8_26,
        23,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        21,
        PLIC_gpio1_bit9_or_gpio2_bit23_IRQHandler
    );
    impl_gpio_pin!(
        P8_27,
        24,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        22,
        PLIC_gpio1_bit10_or_gpio2_bit24_IRQHandler
    );
    impl_gpio_pin!(
        P8_28,
        25,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        23,
        PLIC_gpio1_bit11_or_gpio2_bit25_IRQHandler
    );
    impl_gpio_pin!(
        P8_29,
        26,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        24,
        PLIC_gpio1_bit12_or_gpio2_bit26_IRQHandler
    );
    impl_gpio_pin!(
        P8_30,
        27,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        25,
        PLIC_gpio1_bit13_or_gpio2_bit27_IRQHandler
    );
    // TODO: Figure out FPGA interrupt handlers
    impl_gpio_pin!(
        P8_31,
        0,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        26,
        PLIC_f2m_8_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_8_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_32,
        1,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        27,
        PLIC_f2m_9_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_9_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_33,
        2,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        28,
        PLIC_f2m_10_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_10_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_34,
        3,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        29,
        PLIC_f2m_11_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_11_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_35,
        4,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        30,
        PLIC_f2m_12_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_12_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_36,
        5,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        31,
        PLIC_f2m_13_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_13_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_37,
        6,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        32,
        PLIC_f2m_14_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_14_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_38,
        7,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        33,
        PLIC_f2m_15_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_15_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_39,
        8,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        34,
        PLIC_f2m_16_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_16_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_40,
        9,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        35,
        PLIC_f2m_17_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_17_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_41,
        10,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        36,
        PLIC_f2m_18_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_18_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_42,
        11,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        37,
        PLIC_f2m_19_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_19_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_43,
        12,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        38,
        PLIC_f2m_20_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_20_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_44,
        13,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        39,
        PLIC_f2m_21_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_21_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_45,
        14,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        40,
        PLIC_f2m_22_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_22_INT_OFFSET
    );
    impl_gpio_pin!(
        P8_46,
        15,
        GpioPeripheral::FpgaCore(P8_CORE_GPIO),
        41,
        PLIC_f2m_23_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_23_INT_OFFSET
    );
    impl_gpio_pin!(
        P9_12,
        1,
        GpioPeripheral::FpgaCore(P9_CORE_GPIO),
        42,
        PLIC_f2m_25_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_25_INT_OFFSET
    );
    impl_gpio_pin!(
        P9_15,
        4,
        GpioPeripheral::FpgaCore(P9_CORE_GPIO),
        43,
        PLIC_f2m_28_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_28_INT_OFFSET
    );
    impl_gpio_pin!(
        P9_23,
        10,
        GpioPeripheral::FpgaCore(P9_CORE_GPIO),
        44,
        PLIC_f2m_34_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_34_INT_OFFSET
    );
    impl_gpio_pin!(
        P9_25,
        12,
        GpioPeripheral::FpgaCore(P9_CORE_GPIO),
        45,
        PLIC_f2m_36_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_36_INT_OFFSET
    );
    impl_gpio_pin!(
        P9_27,
        14,
        GpioPeripheral::FpgaCore(P9_CORE_GPIO),
        46,
        PLIC_f2m_38_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_38_INT_OFFSET
    );
    impl_gpio_pin!(
        P9_30,
        17,
        GpioPeripheral::FpgaCore(P9_CORE_GPIO),
        47,
        PLIC_f2m_41_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_41_INT_OFFSET
    );
    impl_gpio_pin!(
        P9_41,
        19,
        GpioPeripheral::FpgaCore(P9_CORE_GPIO),
        48,
        PLIC_f2m_43_IRQHandler,
        pac::PLIC_IRQn_Type_PLIC_F2M_43_INT_OFFSET
    );

    //-------------------------------------------------------------
    use embedded_hal::digital::{OutputPin, PinState};

    pub struct Leds {
        pub led0: Output,
        pub led1: Output,
        pub led2: Output,
        pub led3: Output,
        pub led4: Output,
        pub led5: Output,
        pub led6: Output,
        pub led7: Output,
        pub led8: Output,
        pub led9: Output,
        pub led10: Output,
        pub led11: Output,
    }

    impl Leds {
        pub fn set_led(&mut self, led_num: usize, on: bool) {
            let pin_state = if on { PinState::High } else { PinState::Low };
            match led_num {
                0 => self.led0.set_state(pin_state),
                1 => self.led1.set_state(pin_state),
                2 => self.led2.set_state(pin_state),
                3 => self.led3.set_state(pin_state),
                4 => self.led4.set_state(pin_state),
                5 => self.led5.set_state(pin_state),
                6 => self.led6.set_state(pin_state),
                7 => self.led7.set_state(pin_state),
                8 => self.led8.set_state(pin_state),
                9 => self.led9.set_state(pin_state),
                10 => self.led10.set_state(pin_state),
                11 => self.led11.set_state(pin_state),
                _ => panic!("Invalid LED number: {}", led_num),
            }
            .unwrap();
        }
    }

    impl crate::Peripheral for Leds {
        fn take() -> Option<Self> {
            let led0 = P8_3::take();
            let led1 = P8_4::take();
            let led2 = P8_5::take();
            let led3 = P8_6::take();
            let led4 = P8_7::take();
            let led5 = P8_8::take();
            let led6 = P8_9::take();
            let led7 = P8_10::take();
            let led8 = P8_11::take();
            let led9 = P8_12::take();
            let led10 = P8_13::take();
            let led11 = P8_14::take();
            if let (
                Some(led0),
                Some(led1),
                Some(led2),
                Some(led3),
                Some(led4),
                Some(led5),
                Some(led6),
                Some(led7),
                Some(led8),
                Some(led9),
                Some(led10),
                Some(led11),
            ) = (
                led0, led1, led2, led3, led4, led5, led6, led7, led8, led9, led10, led11,
            ) {
                Some(Self {
                    led0: Output::new(led0),
                    led1: Output::new(led1),
                    led2: Output::new(led2),
                    led3: Output::new(led3),
                    led4: Output::new(led4),
                    led5: Output::new(led5),
                    led6: Output::new(led6),
                    led7: Output::new(led7),
                    led8: Output::new(led8),
                    led9: Output::new(led9),
                    led10: Output::new(led10),
                    led11: Output::new(led11),
                })
            } else {
                None
            }
        }

        unsafe fn steal() -> Self {
            Self {
                led0: Output::new(P8_3::steal()),
                led1: Output::new(P8_4::steal()),
                led2: Output::new(P8_5::steal()),
                led3: Output::new(P8_6::steal()),
                led4: Output::new(P8_7::steal()),
                led5: Output::new(P8_8::steal()),
                led6: Output::new(P8_9::steal()),
                led7: Output::new(P8_10::steal()),
                led8: Output::new(P8_11::steal()),
                led9: Output::new(P8_12::steal()),
                led10: Output::new(P8_13::steal()),
                led11: Output::new(P8_14::steal()),
            }
        }
    }

    //-------------------------------------------------------------

    // SD card detect interrupt handler
    #[doc(hidden)]
    pub const SD_DETECT_INTERRUPT_IDX: u8 = 49;

    #[no_mangle]
    extern "C" fn PLIC_gpio1_bit17_or_gpio2_bit31_IRQHandler() -> u8 {
        let interrupt = unsafe { &mut GPIO_INTERRUPTS[SD_DETECT_INTERRUPT_IDX as usize] };
        if let Some(waker) = interrupt.waker.take() {
            interrupt.triggered = true;
            waker.wake();
        }
        unsafe {
            pac::MSS_GPIO_clear_irq(pac::GPIO2_LO, 31);
            pac::MSS_GPIO_disable_irq(pac::GPIO2_LO, 31);
        }
        pac::EXT_IRQ_DISABLE as u8
    }

    // The "User button" is GPIO0[13]
    // Because we want to have interrupts available on all GPIO2 pins, we can't also have an interrupt on GPIO0[13] due to the fact that the MPFS multiplexes the GPIO0 and GPIO2 interrupts.
    // From mss_gpio.c:
    /*
     * Lookup table of GPIO interrupt number indexed on GPIO ID.
     * The GPIO interrupts are multiplexed. Total GPIO interrupts are 41.
     * 41 = (14 from GPIO0 + 24 from GPIO1 + 3 non direct interrupts)
     * GPIO2 interrupts are not available by default. Setting the corresponding bit
     * in GPIO_INTERRUPT_FAB_CR(31:0) will enable GPIO2(31:0) corresponding
     * interrupt on PLIC.
     */
    // Because of this, we're not bothering to implement the "User button"
}
