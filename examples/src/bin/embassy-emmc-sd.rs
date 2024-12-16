#![no_std]
#![no_main]
// This example demonstrates how to initialize and read from both an eMMC and an SD card.
// It is based off of https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples/blob/main/driver-examples/mss/mss-mmc/mpfs-emmc-sd-write-read/src/application/hart1/u54_1.c

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use mpfs_hal::pac;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut buffer: [u32; 512] = [0; 512];

    println!("Starting eMMC init");
    init_emmc().expect("Failed to initialize EMMC card");
    println!("eMMC card ready!");

    /* Read from eMMC */
    unsafe {
        let status = pac::MSS_MMC_single_block_read(0, buffer.as_mut_ptr() as *mut u32) as u32;
        if status != pac::mss_mmc_status_MSS_MMC_TRANSFER_SUCCESS {
            println!("eMMC read failed with status: {}", status);
        } else {
            println!(
                "eMMC read success, got data (first 32 bytes): {:x?}",
                &buffer[..32]
            );
        }
    }

    println!("Starting SD init");
    init_sd().expect("Failed to initialize SD card");
    println!("SD card ready!");

    /* Read from SD */
    unsafe {
        let status = pac::MSS_MMC_single_block_read(0, buffer.as_mut_ptr() as *mut u32) as u32;
        if status != pac::mss_mmc_status_MSS_MMC_TRANSFER_SUCCESS {
            println!("SD read failed with status: {}", status);
        } else {
            println!(
                "SD read success, got data (first 32 bytes): {:x?}",
                &buffer[..32]
            );
        }
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
    InitFailed(u32),
}

fn init_emmc() -> Result<(), Error> {
    unsafe {
        /* DMA init for eMMC */
        pac::MSS_MPU_configure(
            pac::mss_mpu_mport_t_MSS_MPU_MMC,
            pac::mss_mpu_pmp_region_t_MSS_MPU_PMP_REGION3,
            0x08000000,
            0x200000,
            (pac::MPU_MODE_READ_ACCESS | pac::MPU_MODE_WRITE_ACCESS | pac::MPU_MODE_EXEC_ACCESS)
                as u8,
            pac::mss_mpu_addrm_t_MSS_MPU_AM_NAPOT,
            0,
        );

        if pac::mss_does_xml_ver_support_switch() != 1 {
            return Err(Error::SDSwitchNotSupported);
        }

        if pac::switch_mssio_config(pac::MSS_IO_OPTIONS__EMMC_MSSIO_CONFIGURATION) == 0 {
            return Err(Error::SDSwitchNotSupported);
        }
        let config = pac::mss_mmc_cfg_t {
            card_type: pac::MSS_MMC_CARD_TYPE_MMC as u8,
            data_bus_width: pac::MSS_MMC_DATA_WIDTH_8BIT as u8,
            bus_speed_mode: pac::MSS_MMC_MODE_SDR as u8,
            clk_rate: pac::MSS_MMC_CLOCK_50MHZ,
            bus_voltage: pac::MSS_MMC_1_8V_BUS_VOLTAGE as u8,
        };
        mmc_reset_block();

        let ret_status = pac::MSS_MMC_init(&config);
        if ret_status != pac::mss_mmc_status_MSS_MMC_INIT_SUCCESS {
            return Err(Error::InitFailed(ret_status));
        }
        Ok(())
    }
}

fn init_sd() -> Result<(), Error> {
    unsafe {
        if pac::mss_does_xml_ver_support_switch() != 1 {
            return Err(Error::SDSwitchNotSupported);
        }

        if pac::switch_mssio_config(pac::MSS_IO_OPTIONS__SD_MSSIO_CONFIGURATION) == 0 {
            return Err(Error::SDSwitchNotSupported);
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
            return Err(Error::InitFailed(ret_status));
        }
        Ok(())
    }
}

unsafe fn mmc_reset_block() {
    (*pac::SYSREG).SUBBLK_CLOCK_CR |= pac::SUBBLK_CLOCK_CR_MMC_MASK;
    (*pac::SYSREG).SOFT_RESET_CR |= pac::SOFT_RESET_CR_MMC_MASK;
    (*pac::SYSREG).SOFT_RESET_CR &= !pac::SOFT_RESET_CR_MMC_MASK;
}
