#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use mpfs_hal::pac;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Starting SD card init");

    if let Err(e) = init_sd() {
        println!("Failed to initialize SD card: {:?}", e);
        panic!("Ahhh")
    } else {
        println!("SD card ready!");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}

#[derive(Debug, PartialEq)]
enum Error {
    SDSwitchNotSupported,
    SDInitFailed,
}

fn init_sd() -> Result<(), Error> {
    unsafe {
        if pac::mss_does_xml_ver_support_switch() == 1 {
            return Err(Error::SDSwitchNotSupported);
        }

        while pac::switch_mssio_config(pac::MSS_IO_OPTIONS__SD_MSSIO_CONFIGURATION) == 0 {
            core::hint::spin_loop();
        }

        let config = pac::mss_mmc_cfg_t {
            card_type: pac::MSS_MMC_CARD_TYPE_SD as u8,
            data_bus_width: pac::MSS_MMC_DATA_WIDTH_4BIT as u8,
            bus_speed_mode: pac::MSS_SDCARD_MODE_HIGH_SPEED as u8,
            clk_rate: pac::MSS_MMC_CLOCK_50MHZ,
            bus_voltage: pac::MSS_MMC_3_3V_BUS_VOLTAGE as u8,
        };
        mmc_reset_block();

        let ret_status = pac::MSS_MMC_init(&config);
        if ret_status != pac::mss_mmc_status_MSS_MMC_INIT_SUCCESS {
            return Err(Error::SDInitFailed);
        }
        Ok(())
    }
}

unsafe fn mmc_reset_block() {
    (*pac::SYSREG).SUBBLK_CLOCK_CR |= pac::SUBBLK_CLOCK_CR_MMC_MASK;
    (*pac::SYSREG).SOFT_RESET_CR |= pac::SOFT_RESET_CR_MMC_MASK;
    (*pac::SYSREG).SOFT_RESET_CR &= !pac::SOFT_RESET_CR_MMC_MASK;
}
