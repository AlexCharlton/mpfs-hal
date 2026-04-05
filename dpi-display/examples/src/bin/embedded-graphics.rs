#![no_std]
#![no_main]

// Adapted from https://github.com/embedded-graphics/examples/blob/main/eg-0.8/examples/text-fonts.rs

extern crate alloc;

use dpi_display::{Display, Pixel};
use embedded_graphics::{
    mono_font::{
        ascii::{FONT_10X20, FONT_5X8, FONT_6X12, FONT_9X15},
        MonoTextStyle, MonoTextStyleBuilder,
    },
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    log::info!("Hello world!");

    let mut display = Display::take().unwrap();
    // Make sure the next buffer is available
    let _ = display.get_buffer().await;

    // Set a blue background
    let fill = PrimitiveStyle::with_fill(Pixel::BLUE);
    Rectangle::new(
        Point::new(0, 0),
        Size::new(dpi_display::WIDTH as u32, dpi_display::HEIGHT as u32),
    )
    .into_styled(fill)
    .draw(&mut display)
    .unwrap();

    // Show smallest font
    Text::new(
        "Hello World! - default style 5x8",
        Point::new(15, 15),
        MonoTextStyle::new(&FONT_5X8, Pixel::WHITE),
    )
    .draw(&mut display)
    .unwrap();

    // Show smallest blue font on a white background
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_5X8)
        .text_color(Pixel::BLUE)
        .background_color(Pixel::WHITE)
        .build();

    Text::new("Hello World! - inverse 5x8", Point::new(15, 30), style)
        .draw(&mut display)
        .unwrap();

    // Show 6x12 Font
    Text::new(
        "Hello 6x12!",
        Point::new(15, 45),
        MonoTextStyle::new(&FONT_6X12, Pixel::WHITE),
    )
    .draw(&mut display)
    .unwrap();

    // Show 9x15 Font
    Text::new(
        "Hello 9x15!",
        Point::new(15, 70),
        MonoTextStyle::new(&FONT_9X15, Pixel::WHITE),
    )
    .draw(&mut display)
    .unwrap();

    // Show 10x20 Font
    Text::new(
        "Hello 10x20!",
        Point::new(15, 95),
        MonoTextStyle::new(&FONT_10X20, Pixel::WHITE),
    )
    .draw(&mut display)
    .unwrap();

    display.flush();
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Debug);
    dpi_display::init();
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}
