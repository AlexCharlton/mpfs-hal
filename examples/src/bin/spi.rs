#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

use mpfs_hal::{pac, spi};

#[mpfs_hal::hart1_main]
pub fn hart1_main() {
    println!("Hello World from Rust from hart 1!");

    unsafe {
        pac::MSS_SPI_configure_master_mode(
            &raw mut pac::g_mss_spi0_lo,
            pac::__mss_spi_slave_t_MSS_SPI_SLAVE_0,
            pac::__mss_spi_protocol_mode_t_MSS_SPI_MODE2,
            2,
            pac::MSS_SPI_BLOCK_TRANSFER_FRAME_SIZE as u8,
            Some(mss_spi_overflow_handler),
        );

        pac::MSS_SPI_set_slave_select(
            &raw mut pac::g_mss_spi0_lo,
            pac::__mss_spi_slave_t_MSS_SPI_SLAVE_0,
        );
        pac::MSS_SPI_transfer_frame(&raw mut pac::g_mss_spi0_lo, 0xaa);
        pac::MSS_SPI_clear_slave_select(
            &raw mut pac::g_mss_spi0_lo,
            pac::__mss_spi_slave_t_MSS_SPI_SLAVE_0,
        );
    }

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

extern "C" fn mss_spi_overflow_handler(mss_spi_core: u8) {
    unsafe {
        if mss_spi_core != 0 {
            // reset SPI1
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI1,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_OFF,
            );
            // Take SPI1 out of reset
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI1,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
            );
        } else {
            // reset SPI0
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI0,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_OFF,
            );
            // Take SPI0 out of reset
            pac::mss_config_clk_rst(
                pac::mss_peripherals__MSS_PERIPH_SPI0,
                pac::MPFS_HAL_FIRST_HART as u8,
                pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
            );
        }
    }
}
