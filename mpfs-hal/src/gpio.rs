//! The MPFS broadly uses 2 GPIO interfaces: the MSS GPIO interface and the FPGA Core GPIO interface.
//!
//! By default, on the BeagleV-Fire, P8_3 through P8_30 use the MSS GPIO interface. The remainder use the FPGA Core GPIO interface. Both interfaces are routed through the FPGA and thus depend on the FPGA configuration to work correctly. See the [Default cape gateware](https://git.beagleboard.org/beaglev-fire/gateware/-/tree/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT) for the expected configuration. Note that if you want to use interrupts for the Core GPIO interface, "fixed config" needs to be disabled, or the interrupts need to be otherwise configured in the gateware. The [CoreGPIO_LCD.tcl](https://git.beagleboard.org/beaglev-fire/gateware/-/blob/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT/CoreGPIO_LCD.tcl?ref_type=heads#L7) file controls this config for the P8 pins, while [CoreGPIO_P9.tcl](https://git.beagleboard.org/beaglev-fire/gateware/-/blob/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT/CoreGPIO_P9.tcl?ref_type=heads)
//!
//! This module provides a unified interface to both GPIO interfaces. It also provides a way to define custom Pins and Input/Output peripherals so that any user can define their own GPIO depending on the given FPGA programming.

extern crate alloc;

use paste::paste;

use core::convert::Infallible;
use core::future::Future;
use core::task::{Context, Poll, Waker};

use crate::Peripheral;
use crate::pac;

const MSS_GPIO_INTERRUPT_COUNT: usize = 38;
unsafe fn base_init() {
    unsafe {
        pac::MSS_GPIO_init(pac::GPIO0_LO);
        pac::MSS_GPIO_init(pac::GPIO1_LO);
        pac::MSS_GPIO_init(pac::GPIO2_LO);
        // Interrupts are configured for GPIO0 and GPIO1 by default
        // GPIO2 interrupts are initialized when a Pin is taken
        (*pac::SYSREG).GPIO_INTERRUPT_FAB_CR = 0x0;

        for i in 0..MSS_GPIO_INTERRUPT_COUNT {
            pac::PLIC_SetPriority(
                pac::PLIC_IRQn_Type_PLIC_GPIO0_BIT0_or_GPIO2_BIT0_INT_OFFSET + i as u32,
                2,
            );
        }
        for i in
            pac::PLIC_IRQn_Type_PLIC_F2M_0_INT_OFFSET..=pac::PLIC_IRQn_Type_PLIC_F2M_63_INT_OFFSET
        {
            pac::PLIC_SetPriority(i, 2);
        }
    }
}

//-------------------------------------
// MARK: Interrupts
//-------------------------------------

#[derive(Default, Debug)]
struct GpioInterruptData {
    waker: Option<Waker>,
    triggered: bool,
}

const NUM_INTERRUPTS: usize = 38 + 64; // 38 MSS GPIO interrupts + 64 F2M interrupts
static mut GPIO_INTERRUPTS: [GpioInterruptData; NUM_INTERRUPTS] = [const {
    GpioInterruptData {
        waker: None,
        triggered: false,
    }
}; NUM_INTERRUPTS];

#[repr(u8)]
#[derive(Default, Copy, Clone, Debug, PartialEq)]
#[doc(hidden)]
pub enum InterruptTrigger {
    #[default]
    LevelHigh = pac::GPIO_IRQ_LEVEL_HIGH as u8,
    LevelLow = pac::GPIO_IRQ_LEVEL_LOW as u8,
    EdgePositive = pac::GPIO_IRQ_EDGE_POSITIVE as u8,
    EdgeNegative = pac::GPIO_IRQ_EDGE_NEGATIVE as u8,
    EdgeBoth = pac::GPIO_IRQ_EDGE_BOTH as u8,
}

#[doc(hidden)]
#[derive(Debug, Clone, Copy)]
pub struct Interrupt {
    interrupt_idx: u8,
    plic_idx: u8,
}

impl Interrupt {
    pub const NONE: Self = Self {
        interrupt_idx: 255,
        plic_idx: 255,
    };

    unsafe fn triggered(&self) -> bool {
        unsafe { GPIO_INTERRUPTS[self.interrupt_idx as usize].triggered }
    }

    unsafe fn set_waker(&mut self, waker: Waker) {
        unsafe {
            GPIO_INTERRUPTS[self.interrupt_idx as usize].waker = Some(waker);
        }
    }

    fn is_none(&self) -> bool {
        self.interrupt_idx == 255 && self.plic_idx == 255
    }
}

pub trait GpioInterrupt: 'static {
    #[doc(hidden)]
    fn address(&self) -> Interrupt;
}

macro_rules! impl_gpio_interrupt {
    ($interrupt:ident, $interrupt_idx:expr, $interrupt_handler:ident, $plic_idx:expr) => {
        paste! {
            impl_gpio_interrupt!($interrupt, [<$interrupt _TAKEN>], $interrupt_idx, $interrupt_handler, $plic_idx);
        }
    };

    ($interrupt:ident, $interrupt_idx:expr, $interrupt_handler:ident) => {
        impl_gpio_interrupt!($interrupt, $interrupt_idx, $interrupt_handler, 255);
    };

    ($INTERRUPT:ident, $INTERRUPT_TAKEN:ident, $interrupt_idx:expr, $interrupt_handler:ident, $plic_idx:expr) => {
        #[allow(non_camel_case_types)]
        pub struct $INTERRUPT {
            _private: (),
        }
        static mut $INTERRUPT_TAKEN: bool = false;

        impl crate::Peripheral for $INTERRUPT {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $INTERRUPT_TAKEN {
                        None
                    } else {
                        $INTERRUPT_TAKEN = true;
                        Some(Self { _private: () })
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }

        impl GpioInterrupt for $INTERRUPT {
            fn address(&self) -> Interrupt {
                Interrupt {
                    interrupt_idx: $interrupt_idx,
                    plic_idx: $plic_idx as u8,
                }
            }
        }

        #[unsafe(no_mangle)]
        extern "C" fn $interrupt_handler() -> u8 {
            let interrupt = unsafe { &mut GPIO_INTERRUPTS[$interrupt_idx] };
            trace!("GPIO interrupt {} triggered: {:?}", $interrupt_idx, interrupt);
            if let Some(waker) = interrupt.waker.take() {
                interrupt.triggered = true;
                waker.wake();
            }
            pac::EXT_IRQ_DISABLE as u8
        }
    };
}

impl_gpio_interrupt!(
    GPIO0_0_OR_GPIO2_0_INT,
    0,
    // The first 14 GPIO2 handlers are misnamed
    // This one handles GPIO2[0]
    PLIC_gpio0_bit0_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_1_OR_GPIO2_1_INT,
    1,
    PLIC_gpio0_bit1_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_2_OR_GPIO2_2_INT,
    2,
    PLIC_gpio0_bit2_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_3_OR_GPIO2_3_INT,
    3,
    PLIC_gpio0_bit3_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_4_OR_GPIO2_4_INT,
    4,
    PLIC_gpio0_bit4_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_5_OR_GPIO2_5_INT,
    5,
    PLIC_gpio0_bit5_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_6_OR_GPIO2_6_INT,
    6,
    PLIC_gpio0_bit6_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_7_OR_GPIO2_7_INT,
    7,
    PLIC_gpio0_bit7_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_8_OR_GPIO2_8_INT,
    8,
    PLIC_gpio0_bit8_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_9_OR_GPIO2_9_INT,
    9,
    PLIC_gpio0_bit9_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_10_OR_GPIO2_10_INT,
    10,
    PLIC_gpio0_bit10_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_11_OR_GPIO2_11_INT,
    11,
    PLIC_gpio0_bit11_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_12_OR_GPIO2_12_INT,
    12,
    PLIC_gpio0_bit12_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO0_13_OR_GPIO2_13_INT,
    13,
    PLIC_gpio0_bit13_or_gpio2_bit13_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_0_OR_GPIO2_14_INT,
    14,
    PLIC_gpio1_bit0_or_gpio2_bit14_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_1_OR_GPIO2_15_INT,
    15,
    PLIC_gpio1_bit1_or_gpio2_bit15_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_2_OR_GPIO2_16_INT,
    16,
    PLIC_gpio1_bit2_or_gpio2_bit16_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_3_OR_GPIO2_17_INT,
    17,
    PLIC_gpio1_bit3_or_gpio2_bit17_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_4_OR_GPIO2_18_INT,
    18,
    PLIC_gpio1_bit4_or_gpio2_bit18_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_5_OR_GPIO2_19_INT,
    19,
    PLIC_gpio1_bit5_or_gpio2_bit19_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_6_OR_GPIO2_20_INT,
    20,
    PLIC_gpio1_bit6_or_gpio2_bit20_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_7_OR_GPIO2_21_INT,
    21,
    PLIC_gpio1_bit7_or_gpio2_bit21_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_8_OR_GPIO2_22_INT,
    22,
    PLIC_gpio1_bit8_or_gpio2_bit22_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_9_OR_GPIO2_23_INT,
    23,
    PLIC_gpio1_bit9_or_gpio2_bit23_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_10_OR_GPIO2_24_INT,
    24,
    PLIC_gpio1_bit10_or_gpio2_bit24_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_11_OR_GPIO2_25_INT,
    25,
    PLIC_gpio1_bit11_or_gpio2_bit25_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_12_OR_GPIO2_26_INT,
    26,
    PLIC_gpio1_bit12_or_gpio2_bit26_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_13_OR_GPIO2_27_INT,
    27,
    PLIC_gpio1_bit13_or_gpio2_bit27_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_14_OR_GPIO2_28_INT,
    28,
    PLIC_gpio1_bit14_or_gpio2_bit28_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_15_OR_GPIO2_29_INT,
    29,
    PLIC_gpio1_bit15_or_gpio2_bit29_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_16_OR_GPIO2_30_INT,
    30,
    PLIC_gpio1_bit16_or_gpio2_bit30_IRQHandler
);
impl_gpio_interrupt!(
    GPIO1_17_OR_GPIO2_31_INT,
    31,
    PLIC_gpio1_bit17_or_gpio2_bit31_IRQHandler
);
impl_gpio_interrupt!(GPIO1_18_INT, 32, PLIC_gpio1_bit18_IRQHandler);
impl_gpio_interrupt!(GPIO1_19_INT, 33, PLIC_gpio1_bit19_IRQHandler);
impl_gpio_interrupt!(GPIO1_20_INT, 34, PLIC_gpio1_bit20_IRQHandler);
impl_gpio_interrupt!(GPIO1_21_INT, 35, PLIC_gpio1_bit21_IRQHandler);
impl_gpio_interrupt!(GPIO1_22_INT, 36, PLIC_gpio1_bit22_IRQHandler);
impl_gpio_interrupt!(GPIO1_23_INT, 37, PLIC_gpio1_bit23_IRQHandler);

macro_rules! impl_f2m_interrupt {
    ($num:expr) => {
        paste! {
            impl_gpio_interrupt!([<F2M_ $num _INT>], 38 + $num, [<PLIC_f2m_ $num _IRQHandler>], pac::[<PLIC_IRQn_Type_PLIC_F2M_ $num _INT_OFFSET>]);
        }
    };
}

impl_f2m_interrupt!(0);
impl_f2m_interrupt!(1);
impl_f2m_interrupt!(2);
impl_f2m_interrupt!(3);
impl_f2m_interrupt!(4);
impl_f2m_interrupt!(5);
impl_f2m_interrupt!(6);
impl_f2m_interrupt!(7);
impl_f2m_interrupt!(8);
impl_f2m_interrupt!(9);
impl_f2m_interrupt!(10);
impl_f2m_interrupt!(11);
impl_f2m_interrupt!(12);
impl_f2m_interrupt!(13);
impl_f2m_interrupt!(14);
impl_f2m_interrupt!(15);
impl_f2m_interrupt!(16);
impl_f2m_interrupt!(17);
impl_f2m_interrupt!(18);
impl_f2m_interrupt!(19);
impl_f2m_interrupt!(20);
impl_f2m_interrupt!(21);
impl_f2m_interrupt!(22);
impl_f2m_interrupt!(23);
impl_f2m_interrupt!(24);
impl_f2m_interrupt!(25);
impl_f2m_interrupt!(26);
impl_f2m_interrupt!(27);
impl_f2m_interrupt!(28);
impl_f2m_interrupt!(29);
impl_f2m_interrupt!(30);
impl_f2m_interrupt!(31);
impl_f2m_interrupt!(32);
impl_f2m_interrupt!(33);
impl_f2m_interrupt!(34);
impl_f2m_interrupt!(35);
impl_f2m_interrupt!(36);
impl_f2m_interrupt!(37);
impl_f2m_interrupt!(38);
impl_f2m_interrupt!(39);
impl_f2m_interrupt!(40);
impl_f2m_interrupt!(41);
impl_f2m_interrupt!(42);
impl_f2m_interrupt!(43);
impl_f2m_interrupt!(44);
impl_f2m_interrupt!(45);
impl_f2m_interrupt!(46);
impl_f2m_interrupt!(47);
impl_f2m_interrupt!(48);
impl_f2m_interrupt!(49);
impl_f2m_interrupt!(50);
impl_f2m_interrupt!(51);
impl_f2m_interrupt!(52);
impl_f2m_interrupt!(53);
impl_f2m_interrupt!(54);
impl_f2m_interrupt!(55);
impl_f2m_interrupt!(56);
impl_f2m_interrupt!(57);
impl_f2m_interrupt!(58);
impl_f2m_interrupt!(59);
impl_f2m_interrupt!(60);
impl_f2m_interrupt!(61);
impl_f2m_interrupt!(62);
impl_f2m_interrupt!(63);

//-------------------------------------
// MARK: Pins
//-------------------------------------

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
#[doc(hidden)]
pub enum GpioPeripheral {
    Mss(*mut pac::GPIO_TypeDef),
    FpgaCore(pac::gpio_instance_t),
}

impl GpioPeripheral {
    pub fn is_gpio2(&self) -> bool {
        if let GpioPeripheral::Mss(gpio) = self {
            gpio == &pac::GPIO2_LO
        } else {
            false
        }
    }
}

#[doc(hidden)]
#[derive(Debug, Clone, Copy)]
pub struct Pin {
    number: u8,
    peripheral: GpioPeripheral,
    interrupt: Interrupt,
}

impl Pin {
    // Meant to be used only by board-support code
    #[doc(hidden)]
    pub fn new(number: u8, peripheral: GpioPeripheral, interrupt: Interrupt) -> Self {
        Self {
            number,
            peripheral,
            interrupt,
        }
    }

    #[doc(hidden)]
    pub fn config_output(&self) {
        unsafe {
            match self.peripheral {
                GpioPeripheral::Mss(typedef) => {
                    trace!("Configuring MSS GPIO {:?} to output", self);
                    pac::MSS_GPIO_config(typedef, self.number as u32, pac::MSS_GPIO_OUTPUT_MODE);
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    trace!("Configuring FPGA Core GPIO {:?} to output", self);
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
                    trace!(
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
                    trace!(
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
                    trace!("Setting MSS GPIO {:?} to high", self);
                    pac::MSS_GPIO_set_output(typedef, self.number as u32, 1);
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    let mut gpio_outputs = pac::GPIO_get_outputs(&mut address);
                    gpio_outputs |= 1 << self.number;
                    trace!(
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
                    trace!("Setting MSS GPIO {:?} to low", self);
                    pac::MSS_GPIO_set_output(typedef, self.number as u32, 0);
                }
                GpioPeripheral::FpgaCore(address) => {
                    let mut address = address;
                    let mut gpio_outputs = pac::GPIO_get_outputs(&mut address);
                    gpio_outputs &= !(1 << self.number);
                    trace!(
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
                    pac::PLIC_EnableIRQ(self.interrupt.plic_idx as u32);
                }
            }
        }
    }

    fn clear_triggered(&self) {
        critical_section::with(|_| unsafe {
            GPIO_INTERRUPTS[self.interrupt.interrupt_idx as usize].triggered = false;
        });
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
        if address.interrupt.is_none() {
            panic!(
                "Cannot create an input pin with no interrupt: {:?}",
                address
            );
        }
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
    trigger: InterruptTrigger,
}

impl InputFuture {
    pub fn new(pin: Pin, interrupt_trigger: InterruptTrigger) -> Self {
        critical_section::with(|_| unsafe {
            GPIO_INTERRUPTS[pin.interrupt.interrupt_idx as usize].triggered = false;
        });
        pin.config_input(interrupt_trigger);
        pin.enable_interrupt();
        Self {
            pin,
            trigger: interrupt_trigger,
        }
    }
}

impl Future for InputFuture {
    type Output = Result<(), Infallible>;

    fn poll(mut self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let trigger = critical_section::with(|_| unsafe {
            self.as_mut().pin.interrupt.set_waker(cx.waker().clone());
            self.as_mut().pin.interrupt.triggered()
        });

        if ((self.trigger == InterruptTrigger::EdgePositive
            || self.trigger == InterruptTrigger::LevelHigh)
            && self.as_mut().pin.is_high())
            || ((self.trigger == InterruptTrigger::EdgeNegative
                || self.trigger == InterruptTrigger::LevelLow)
                && !self.as_mut().pin.is_high())
        {
            // Check to see if the pin is in the state we're looking for, even if the interrupt wasn't triggered
            Poll::Ready(Ok(()))
        } else if trigger {
            if ((self.trigger == InterruptTrigger::EdgePositive
                || self.trigger == InterruptTrigger::LevelHigh)
                && !self.as_mut().pin.is_high())
                || ((self.trigger == InterruptTrigger::EdgeNegative
                    || self.trigger == InterruptTrigger::LevelLow)
                    && self.as_mut().pin.is_high())
            {
                // The interrupt was triggered, but the pin is not in the correct state
                // so we reset the interrupt and wait for the next one
                self.as_mut().pin.clear_triggered();
                self.as_mut().pin.enable_interrupt();
                return Poll::Pending;
            }
            self.as_mut().pin.clear_triggered();
            Pin::clear_interrupt(self.as_mut().pin.peripheral, self.as_mut().pin.number);
            Pin::disable_interrupt(self.as_mut().pin.peripheral, self.as_mut().pin.number);
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
// MARK: Macros
//----------------------------------------------------------------------

#[macro_export]
/// This macro is used to define a GPIO pin.
///
/// It does not actually guard against defining the same resource multiple times, so be careful.
macro_rules! impl_gpio_pin {
    ($pin:ident, $peripheral:expr, $n:expr, $interrupt:ident) => {
        paste::paste! {
            impl_gpio_pin!($pin, [<$pin _TAKEN>], $peripheral, $n, $interrupt);
        }
    };

    ($PIN:ident, $PIN_TAKEN:ident, $peripheral:expr, $num:expr, NONE) => {
        #[allow(non_camel_case_types)]
        pub struct $PIN {
            _private: (),
        }
        static mut $PIN_TAKEN: bool = false;

        impl $crate::Peripheral for $PIN {
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

        impl $crate::gpio::GpioPin for $PIN {
            fn address(&self) -> $crate::gpio::Pin {
                $crate::gpio::Pin::new($num, $peripheral, $crate::gpio::Interrupt::NONE)
            }
        }
    };

    ($PIN:ident, $PIN_TAKEN:ident, $peripheral:expr, $num:expr, $interrupt:ident) => {
        #[allow(non_camel_case_types)]
        pub struct $PIN {
            _private: (),
        }
        static mut $PIN_TAKEN: bool = false;

        impl $crate::Peripheral for $PIN {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $PIN_TAKEN {
                        None
                    } else {
                        if $interrupt::take().is_some() {
                            $PIN_TAKEN = true;
                            if $peripheral.is_gpio2() {
                                // Enable GPIO2 interrupt
                                (*$crate::pac::SYSREG).GPIO_INTERRUPT_FAB_CR |= 1 << $num;
                                trace!(
                                    "Enabled GPIO2 interrupt for pin {}; {:x?}",
                                    $num,
                                    (*$crate::pac::SYSREG).GPIO_INTERRUPT_FAB_CR
                                );
                            }
                            Some(Self { _private: () })
                        } else {
                            None
                        }
                    }
                })
            }

            unsafe fn steal() -> Self {
                Self { _private: () }
            }
        }

        impl $crate::gpio::GpioPin for $PIN {
            fn address(&self) -> $crate::gpio::Pin {
                $crate::gpio::Pin::new($num, $peripheral, unsafe {
                    $crate::gpio::GpioInterrupt::address(&$interrupt::steal())
                })
            }
        }
    };
}

#[macro_export]
macro_rules! impl_input_peripheral {
    ($perif:ident, $pin:ident) => {
        paste::paste! {
            impl_input_peripheral!($perif, [<$perif _TAKEN>], $pin);
        }
    };

    ($PERIF:ident, $PERIF_TAKEN:ident, $pin:ident) => {
        #[allow(non_camel_case_types)]
        pub struct $PERIF {
            pub pin: $crate::gpio::Input,
        }
        #[allow(non_upper_case_globals)]
        static mut $PERIF_TAKEN: bool = false;

        impl $crate::Peripheral for $PERIF {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $PERIF_TAKEN {
                        None
                    } else {
                        if $pin::take().is_some() {
                            $PERIF_TAKEN = true;
                            Some(Self {
                                pin: $crate::gpio::Input::new($pin::steal()),
                            })
                        } else {
                            None
                        }
                    }
                })
            }

            unsafe fn steal() -> Self {
                unsafe {
                    Self {
                        pin: $crate::gpio::Input::new($pin::steal()),
                    }
                }
            }
        }

        impl embedded_hal::digital::ErrorType for $PERIF {
            type Error = core::convert::Infallible;
        }

        impl embedded_hal::digital::InputPin for $PERIF {
            fn is_high(&mut self) -> Result<bool, Self::Error> {
                self.pin.is_high()
            }

            fn is_low(&mut self) -> Result<bool, Self::Error> {
                self.pin.is_low()
            }
        }

        impl embedded_hal_async::digital::Wait for $PERIF {
            async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
                self.pin.wait_for_high().await
            }

            async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
                self.pin.wait_for_low().await
            }

            async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
                self.pin.wait_for_rising_edge().await
            }

            async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
                self.pin.wait_for_falling_edge().await
            }

            async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
                self.pin.wait_for_any_edge().await
            }
        }
    };
}

#[macro_export]
macro_rules! impl_output_peripheral {
    ($perif:ident, $pin:ident) => {
        paste::paste! {
            impl_output_peripheral!($perif, [<$perif _TAKEN>], $pin);
        }
    };

    ($PERIF:ident, $PERIF_TAKEN:ident, $pin:ident) => {
        #[allow(non_camel_case_types)]
        pub struct $PERIF {
            pub pin: $crate::gpio::Output,
        }
        #[allow(non_upper_case_globals)]
        static mut $PERIF_TAKEN: bool = false;

        impl $crate::Peripheral for $PERIF {
            fn take() -> Option<Self> {
                critical_section::with(|_| unsafe {
                    if $PERIF_TAKEN {
                        None
                    } else {
                        if $pin::take().is_some() {
                            $PERIF_TAKEN = true;
                            Some(Self {
                                pin: $crate::gpio::Output::new($pin::steal()),
                            })
                        } else {
                            None
                        }
                    }
                })
            }

            unsafe fn steal() -> Self {
                unsafe {
                    Self {
                        pin: $crate::gpio::Output::new($pin::steal()),
                    }
                }
            }
        }

        impl embedded_hal::digital::ErrorType for $PERIF {
            type Error = core::convert::Infallible;
        }

        impl embedded_hal::digital::OutputPin for $PERIF {
            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.pin.set_low()
            }

            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.pin.set_high()
            }
        }
    };
}

//----------------------------------------------------------------------
// MARK: Beaglev Fire
//-------------------------------------

#[cfg(feature = "beaglev-fire")]
pub use beaglev_fire::*;
#[cfg(feature = "beaglev-fire")]
mod beaglev_fire {
    use super::*;
    use crate::pac;

    use embedded_hal::digital::InputPin;
    use embedded_hal_async::digital::Wait;
    use mutually_exclusive_features::exactly_one_of;

    exactly_one_of!(
        "beaglev-fire-default-cape",
        "beaglev-fire-gpio-cape",
        "beaglev-fire-no-cape"
    );

    /// To be called before using any GPIO pins
    pub fn init() {
        unsafe {
            base_init();
            #[cfg(any(
                feature = "beaglev-fire-gpio-cape",
                feature = "beaglev-fire-default-cape"
            ))]
            beaglev_fire_default_cape::cape_init();
        }
    }

    // IO that is always available

    impl_gpio_pin!(
        SD_DETECT,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        31,
        GPIO1_17_OR_GPIO2_31_INT
    );
    impl_input_peripheral!(SdDetect, SD_DETECT);

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

    impl_gpio_pin!(SD_CHIP_SELECT, GpioPeripheral::Mss(pac::GPIO0_LO), 12, NONE);
    impl_output_peripheral!(SdChipSelect, SD_CHIP_SELECT);

    impl_gpio_pin!(
        USER_BUTTON,
        GpioPeripheral::Mss(pac::GPIO0_LO),
        13,
        GPIO0_13_OR_GPIO2_13_INT
    );
    impl_input_peripheral!(UserButton, USER_BUTTON);

    //-------------------------------------------------------------
    // Cape GPIOs

    // From the Cape documentation: https://git.beagleboard.org/beaglev-fire/gateware/-/blob/main/sources/FPGA-design/script_support/components/CAPE/DEFAULT/Readme.md
    // https://git.beagleboard.org/beaglev-fire/gateware/-/blob/main/sources/FPGA-design/script_support/components/CAPE/GPIOS/Readme.md
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
    | P8_13  | MSS GPIO_2[10]             |   53  | User LED 10 | // ONLY ON GPIO CAPE
    | P8_14  | MSS GPIO_2[11]             |   53  | User LED 11 |
    | P8_15  | MSS GPIO_2[12]             |   53  | GPIO        |
    | P8_16  | MSS GPIO_2[13]             |   53  | GPIO        |
    | P8_17  | MSS GPIO_2[14]             |   53  | GPIO        |
    | P8_18  | MSS GPIO_2[15]             |   53  | GPIO        |
    | P8_19  | MSS GPIO_2[16]             |   53  | GPIO        | // ONLY ON GPIO CAPE
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
    -------------------------------------------------------------
    | P9_11  | core_gpio[0] @ 0x41200000  |  142  | GPIO        | // ONLY ON GPIO CAPE
    | P9_12  | core_gpio[1] @ 0x41200000  |  143  | GPIO        |
    | P9_13  | core_gpio[2] @ 0x41200000  |  144  | GPIO        | // ONLY ON GPIO CAPE
    | P9_15  | core_gpio[4] @ 0x41200000  |  146  | GPIO        |
    | P9_16  | core_gpio[5] @ 0x41200000  |  147  | GPIO        | // ONLY ON GPIO CAPE
    | P9_17  | core_gpio[6] @ 0x41200000  |  148  | GPIO        | // ONLY ON GPIO CAPE
    | P9_18  | core_gpio[7] @ 0x41200000  |  149  | GPIO        | // ONLY ON GPIO CAPE
    | P9_21  | core_gpio[8] @ 0x41200000  |  150  | GPIO        | // ONLY ON GPIO CAPE
    | P9_22  | core_gpio[9] @ 0x41200000  |  151  | GPIO        | // ONLY ON GPIO CAPE
    | P9_23  | core_gpio[10] @ 0x41200000 |  152  | GPIO        |
    | P9_24  | core_gpio[11] @ 0x41200000 |  153  | GPIO        | // ONLY ON GPIO CAPE
    | P9_25  | core_gpio[12] @ 0x41200000 |  154  | GPIO        |
    | P9_26  | core_gpio[13] @ 0x41200000 |  155  | GPIO        | // ONLY ON GPIO CAPE
    | P9_27  | core_gpio[14] @ 0x41200000 |  156  | GPIO        |
    | P9_28  | core_gpio[15] @ 0x41200000 |  157  | GPIO        | // ONLY ON GPIO CAPE
    | P9_29  | core_gpio[16] @ 0x41200000 |  158  | GPIO        | // ONLY ON GPIO CAPE
    | P9_30  | core_gpio[17] @ 0x41200000 |  159  | GPIO        |
    | P9_31  | core_gpio[18] @ 0x41200000 |  160  | GPIO        | // ONLY ON GPIO CAPE
    | P9_41  | core_gpio[19] @ 0x41200000 |  161  | GPIO        |
    | P9_42  | core_gpio[20] @ 0x41200000 |  162  | GPIO        | // ONLY ON GPIO CAPE
    */

    #[cfg(any(
        feature = "beaglev-fire-gpio-cape",
        feature = "beaglev-fire-default-cape"
    ))]
    pub use beaglev_fire_default_cape::*;

    #[cfg(any(
        feature = "beaglev-fire-gpio-cape",
        feature = "beaglev-fire-default-cape"
    ))]
    mod beaglev_fire_default_cape {
        use super::*;
        use crate::pac;

        pub(crate) unsafe fn cape_init() {
            unsafe {
                // FPGA Core GPIO initialization
                let mut p8: pac::gpio_instance_t = P8_CORE_GPIO;
                trace!("Initializing FPGA Core GPIO {:?}", p8);
                pac::GPIO_init(&mut p8, P8_CORE_GPIO.base_addr, P8_CORE_GPIO.apb_bus_width);
                let mut p9: pac::gpio_instance_t = P9_CORE_GPIO;
                trace!("Initializing FPGA Core GPIO {:?}", p9);
                pac::GPIO_init(&mut p9, P9_CORE_GPIO.base_addr, P9_CORE_GPIO.apb_bus_width);
            }
        }

        pub const P8_CORE_GPIO: pac::gpio_instance_t = pac::gpio_instance_t {
            base_addr: 0x4110_0000,
            apb_bus_width: pac::__gpio_apb_width_t_GPIO_APB_32_BITS_BUS,
        };

        pub const P9_CORE_GPIO: pac::gpio_instance_t = pac::gpio_instance_t {
            base_addr: 0x41200000,
            apb_bus_width: pac::__gpio_apb_width_t_GPIO_APB_32_BITS_BUS,
        };

        impl_gpio_pin!(
            P8_3,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            0,
            GPIO0_0_OR_GPIO2_0_INT
        );
        impl_gpio_pin!(
            P8_4,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            1,
            GPIO0_1_OR_GPIO2_1_INT
        );
        impl_gpio_pin!(
            P8_5,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            2,
            GPIO0_2_OR_GPIO2_2_INT
        );
        impl_gpio_pin!(
            P8_6,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            3,
            GPIO0_3_OR_GPIO2_3_INT
        );
        impl_gpio_pin!(
            P8_7,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            4,
            GPIO0_4_OR_GPIO2_4_INT
        );
        impl_gpio_pin!(
            P8_8,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            5,
            GPIO0_5_OR_GPIO2_5_INT
        );
        impl_gpio_pin!(
            P8_9,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            6,
            GPIO0_6_OR_GPIO2_6_INT
        );
        impl_gpio_pin!(
            P8_10,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            7,
            GPIO0_7_OR_GPIO2_7_INT
        );
        impl_gpio_pin!(
            P8_11,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            8,
            GPIO0_8_OR_GPIO2_8_INT
        );
        impl_gpio_pin!(
            P8_12,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            9,
            GPIO0_9_OR_GPIO2_9_INT
        );
        impl_gpio_pin!(
            P8_14,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            11,
            GPIO0_11_OR_GPIO2_11_INT
        );
        impl_gpio_pin!(
            P8_15,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            12,
            GPIO0_12_OR_GPIO2_12_INT
        );
        impl_gpio_pin!(
            P8_16,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            13,
            GPIO0_13_OR_GPIO2_13_INT
        );
        impl_gpio_pin!(
            P8_17,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            14,
            GPIO1_0_OR_GPIO2_14_INT
        );
        impl_gpio_pin!(
            P8_18,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            15,
            GPIO1_1_OR_GPIO2_15_INT
        );
        impl_gpio_pin!(
            P8_20,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            17,
            GPIO1_3_OR_GPIO2_17_INT
        );
        impl_gpio_pin!(
            P8_21,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            18,
            GPIO1_4_OR_GPIO2_18_INT
        );
        impl_gpio_pin!(
            P8_22,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            19,
            GPIO1_5_OR_GPIO2_19_INT
        );
        impl_gpio_pin!(
            P8_23,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            20,
            GPIO1_6_OR_GPIO2_20_INT
        );
        impl_gpio_pin!(
            P8_24,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            21,
            GPIO1_7_OR_GPIO2_21_INT
        );
        impl_gpio_pin!(
            P8_25,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            22,
            GPIO1_8_OR_GPIO2_22_INT
        );
        impl_gpio_pin!(
            P8_26,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            23,
            GPIO1_9_OR_GPIO2_23_INT
        );
        impl_gpio_pin!(
            P8_27,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            24,
            GPIO1_10_OR_GPIO2_24_INT
        );
        impl_gpio_pin!(
            P8_28,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            25,
            GPIO1_11_OR_GPIO2_25_INT
        );
        impl_gpio_pin!(
            P8_29,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            26,
            GPIO1_12_OR_GPIO2_26_INT
        );
        impl_gpio_pin!(
            P8_30,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            27,
            GPIO1_13_OR_GPIO2_27_INT
        );

        impl_gpio_pin!(P8_31, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 0, F2M_8_INT);
        impl_gpio_pin!(P8_32, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 1, F2M_9_INT);
        impl_gpio_pin!(P8_33, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 2, F2M_10_INT);
        impl_gpio_pin!(P8_34, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 3, F2M_11_INT);
        impl_gpio_pin!(P8_35, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 4, F2M_12_INT);
        impl_gpio_pin!(P8_36, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 5, F2M_13_INT);
        impl_gpio_pin!(P8_37, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 6, F2M_14_INT);
        impl_gpio_pin!(P8_38, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 7, F2M_15_INT);
        impl_gpio_pin!(P8_39, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 8, F2M_16_INT);
        impl_gpio_pin!(P8_40, GpioPeripheral::FpgaCore(P8_CORE_GPIO), 9, F2M_17_INT);
        impl_gpio_pin!(
            P8_41,
            GpioPeripheral::FpgaCore(P8_CORE_GPIO),
            10,
            F2M_18_INT
        );
        impl_gpio_pin!(
            P8_42,
            GpioPeripheral::FpgaCore(P8_CORE_GPIO),
            11,
            F2M_19_INT
        );
        impl_gpio_pin!(
            P8_43,
            GpioPeripheral::FpgaCore(P8_CORE_GPIO),
            12,
            F2M_20_INT
        );
        impl_gpio_pin!(
            P8_44,
            GpioPeripheral::FpgaCore(P8_CORE_GPIO),
            13,
            F2M_21_INT
        );
        impl_gpio_pin!(
            P8_45,
            GpioPeripheral::FpgaCore(P8_CORE_GPIO),
            14,
            F2M_22_INT
        );
        impl_gpio_pin!(
            P8_46,
            GpioPeripheral::FpgaCore(P8_CORE_GPIO),
            15,
            F2M_23_INT
        );
        impl_gpio_pin!(P9_12, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 1, F2M_25_INT);
        impl_gpio_pin!(P9_15, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 4, F2M_28_INT);
        impl_gpio_pin!(
            P9_23,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            10,
            F2M_34_INT
        );
        impl_gpio_pin!(
            P9_25,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            12,
            F2M_36_INT
        );
        impl_gpio_pin!(
            P9_27,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            14,
            F2M_38_INT
        );
        impl_gpio_pin!(
            P9_30,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            17,
            F2M_41_INT
        );
        impl_gpio_pin!(
            P9_41,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            19,
            F2M_43_INT
        );

        impl_output_peripheral!(Led0, P8_3);
        impl_output_peripheral!(Led1, P8_4);
        impl_output_peripheral!(Led2, P8_5);
        impl_output_peripheral!(Led3, P8_6);
        impl_output_peripheral!(Led4, P8_7);
        impl_output_peripheral!(Led5, P8_8);
        impl_output_peripheral!(Led6, P8_9);
        impl_output_peripheral!(Led7, P8_10);
        impl_output_peripheral!(Led8, P8_11);
        impl_output_peripheral!(Led9, P8_12);
        impl_output_peripheral!(Led11, P8_14);

        use embedded_hal::digital::{OutputPin, PinState};
        pub struct Leds {
            pub led0: Led0,
            pub led1: Led1,
            pub led2: Led2,
            pub led3: Led3,
            pub led4: Led4,
            pub led5: Led5,
            pub led6: Led6,
            pub led7: Led7,
            pub led8: Led8,
            pub led9: Led9,
            #[cfg(feature = "beaglev-fire-gpio-cape")]
            pub led10: Led10,
            pub led11: Led11,
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
                    #[cfg(feature = "beaglev-fire-gpio-cape")]
                    10 => self.led10.set_state(pin_state),
                    #[cfg(not(feature = "beaglev-fire-gpio-cape"))]
                    10 => Ok(()),
                    11 => self.led11.set_state(pin_state),
                    _ => panic!("Invalid LED number: {}", led_num),
                }
                .unwrap();
            }
        }

        impl crate::Peripheral for Leds {
            fn take() -> Option<Self> {
                let led0 = Led0::take();
                let led1 = Led1::take();
                let led2 = Led2::take();
                let led3 = Led3::take();
                let led4 = Led4::take();
                let led5 = Led5::take();
                let led6 = Led6::take();
                let led7 = Led7::take();
                let led8 = Led8::take();
                let led9 = Led9::take();
                #[cfg(feature = "beaglev-fire-gpio-cape")]
                let led10 = Led10::take();
                let led11 = Led11::take();

                #[cfg(feature = "beaglev-fire-default-cape")]
                {
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
                        Some(led11),
                    ) = (
                        led0, led1, led2, led3, led4, led5, led6, led7, led8, led9, led11,
                    ) {
                        Some(Self {
                            led0,
                            led1,
                            led2,
                            led3,
                            led4,
                            led5,
                            led6,
                            led7,
                            led8,
                            led9,
                            led11,
                        })
                    } else {
                        None
                    }
                }
                #[cfg(feature = "beaglev-fire-gpio-cape")]
                {
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
                            led0,
                            led1,
                            led2,
                            led3,
                            led4,
                            led5,
                            led6,
                            led7,
                            led8,
                            led9,
                            led10,
                            led11,
                        })
                    } else {
                        None
                    }
                }
            }

            unsafe fn steal() -> Self {
                unsafe {
                    Self {
                        led0: Led0::steal(),
                        led1: Led1::steal(),
                        led2: Led2::steal(),
                        led3: Led3::steal(),
                        led4: Led4::steal(),
                        led5: Led5::steal(),
                        led6: Led6::steal(),
                        led7: Led7::steal(),
                        led8: Led8::steal(),
                        led9: Led9::steal(),
                        #[cfg(feature = "beaglev-fire-gpio-cape")]
                        led10: Led10::steal(),
                        led11: Led11::steal(),
                    }
                }
            }
        }
    }

    #[cfg(feature = "beaglev-fire-gpio-cape")]
    pub use beaglev_fire_gpio_cape::*;

    #[cfg(feature = "beaglev-fire-gpio-cape")]
    mod beaglev_fire_gpio_cape {
        use super::*;
        use crate::pac;
        impl_gpio_pin!(
            P8_13,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            10,
            GPIO0_10_OR_GPIO2_10_INT
        );

        impl_output_peripheral!(Led10, P8_13);

        impl_gpio_pin!(
            P8_19,
            GpioPeripheral::Mss(pac::GPIO2_LO),
            16,
            GPIO1_2_OR_GPIO2_16_INT
        );

        impl_gpio_pin!(P9_11, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 0, F2M_24_INT);
        impl_gpio_pin!(P9_13, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 2, F2M_26_INT);
        impl_gpio_pin!(P9_16, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 5, F2M_29_INT);
        impl_gpio_pin!(P9_17, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 6, F2M_30_INT);
        impl_gpio_pin!(P9_18, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 7, F2M_31_INT);
        impl_gpio_pin!(P9_21, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 8, F2M_32_INT);
        impl_gpio_pin!(P9_22, GpioPeripheral::FpgaCore(P9_CORE_GPIO), 9, F2M_33_INT);
        impl_gpio_pin!(
            P9_24,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            11,
            F2M_35_INT
        );
        impl_gpio_pin!(
            P9_26,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            13,
            F2M_37_INT
        );
        impl_gpio_pin!(
            P9_28,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            15,
            F2M_39_INT
        );
        impl_gpio_pin!(
            P9_29,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            16,
            F2M_40_INT
        );
        impl_gpio_pin!(
            P9_31,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            18,
            F2M_42_INT
        );
        impl_gpio_pin!(
            P9_41,
            GpioPeripheral::FpgaCore(P9_CORE_GPIO),
            19,
            F2M_44_INT
        );
    }
}
