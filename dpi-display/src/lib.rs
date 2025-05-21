#![no_std]

#[cfg(feature = "resolution-800x480")]
mod resolution {
    pub const WIDTH: usize = 800;
    pub const HEIGHT: usize = 480;
}

pub use resolution::*;

use mpfs_hal::{
    gpio::{GpioPeripheral, GPIO1_12_OR_GPIO2_26_INT, GPIO1_13_OR_GPIO2_27_INT},
    impl_gpio_pin, impl_input_peripheral, impl_output_peripheral, pac, Peripheral,
};

impl_gpio_pin!(BUFFER0_READY, GpioPeripheral::Mss(pac::GPIO2_LO), 26, NONE);
impl_output_peripheral!(Buffer0Ready, BUFFER0_READY);

impl_gpio_pin!(BUFFER1_READY, GpioPeripheral::Mss(pac::GPIO2_LO), 27, NONE);
impl_output_peripheral!(Buffer1Ready, BUFFER1_READY);

impl_gpio_pin!(
    BUFFER0_LOCKED,
    GpioPeripheral::Mss(pac::GPIO2_LO),
    26,
    GPIO1_12_OR_GPIO2_26_INT
);
impl_input_peripheral!(Buffer0Locked, BUFFER0_LOCKED);

impl_gpio_pin!(
    BUFFER1_LOCKED,
    GpioPeripheral::Mss(pac::GPIO2_LO),
    27,
    GPIO1_13_OR_GPIO2_27_INT
);
impl_input_peripheral!(Buffer1Locked, BUFFER1_LOCKED);
