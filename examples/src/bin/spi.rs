#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

use aligned::{Aligned, A4};
use mpfs_hal::{
    spi::{self, Spi, Spi0Slave0, SpiConfig},
    Peripheral,
};

use embedded_hal::spi::{Phase, Polarity, SpiDevice};

#[mpfs_hal::hart1_main]
pub fn hart1_main() {
    println!("Hello World from Rust from hart 1!");

    let config = SpiConfig::new(2, Phase::CaptureOnFirstTransition, Polarity::IdleLow).unwrap();
    let mut spi = Spi::new(Spi0Slave0::take().unwrap(), config);

    let mut data = Aligned::<A4, _>([
        0xc0, 0xde, 0xba, 0xbe, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
        0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0xfe, 0xed,
    ]);
    spi.transfer_in_place(data.as_mut_slice()).unwrap();

    println!("Data: {:x?}", data);

    println!("SPI0 transfer done :)");
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Debug);
    spi::init();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
