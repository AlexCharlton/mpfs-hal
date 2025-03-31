#![no_std]
#![no_main]

extern crate alloc;

#[macro_use]
extern crate mpfs_hal;

use alloc::{string::String, vec::Vec};
use block_device_adapters::{BufStream, BufStreamError, StreamSlice};
use embassy_embedded_hal::{shared_bus::asynch::spi::SpiDeviceWithConfig, SetConfig};
use embedded_fatfs::FsOptions;
use embedded_io_async::Read;
use mbr_nostd::{MasterBootRecord, PartitionTable};
use mpfs_hal::{
    qspi::{SpiConfig, SpiFrequency},
    Peripheral,
};
use sdspi::{sd_init, SdSpi};

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello");

    let (mut cs, qspi_bus) = mpfs_hal_embassy::sd::init();
    let mut qspi = qspi_bus.get_mut();
    qspi.set_config(&SpiConfig::default()).unwrap();

    let mut sd_detect = mpfs_hal_embassy::sd::SdDetect::take().unwrap();

    if !sd_detect.is_inserted() {
        println!("SD card not inserted, waiting...");
        sd_detect.wait_for_inserted().await.unwrap();
    }
    println!("SD card detected");

    loop {
        match sd_init(&mut qspi, &mut cs).await {
            Ok(_) => break,
            Err(e) => {
                println!("Sd init error: {:?}", e);
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }
    println!("sd_init complete");

    let spid = SpiDeviceWithConfig::new(qspi_bus, cs, SpiConfig::default());
    let mut sd = SdSpi::<_, _, aligned::A4>::new(spid, embassy_time::Delay);

    while sd.init().await.is_err() {
        println!("Failed to init card, retrying...");
        embassy_time::Timer::after_millis(500).await;
    }
    // Increase the speed up to the SD max of 25mhz
    let mut config = SpiConfig::default();
    config.frequency = SpiFrequency::F25_000_000;
    sd.spi().set_config(config);

    let size = sd.size().await;
    println!("Initialization complete! Got card with size {:?}", size);

    let mut inner = BufStream::<_, 512>::new(sd);
    let mut buf = [0; 512];
    inner.read(&mut buf).await.unwrap();
    let mbr = MasterBootRecord::from_bytes(&buf).unwrap();
    println!("MBR: {:?}\n", mbr.partition_table_entries());

    let partition = mbr.partition_table_entries()[0];
    let start_offset = partition.logical_block_address as u64 * 512;
    let end_offset = start_offset + partition.sector_count as u64 * 512;
    let inner = StreamSlice::new(inner, start_offset, end_offset)
        .await
        .unwrap();

    async {
        let fs = embedded_fatfs::FileSystem::new(inner, FsOptions::new())
            .await
            .unwrap();
        {
            let root = fs.root_dir();
            let mut iter = root.iter();
            while let Some(Ok(entry)) = iter.next().await {
                println!("Name:{} Length:{}", entry.short_file_name(), entry.len());
                if entry.is_file() {
                    let mut file = root.open_file(&entry.short_file_name()).await.unwrap();
                    let buf = read_to_end(&mut file).await.unwrap();
                    println!("  File contents:\n  {}", String::from_utf8_lossy(&buf));
                }
            }
        }
        fs.unmount().await.unwrap();

        Ok::<(), embedded_fatfs::Error<BufStreamError<sdspi::Error>>>(())
    }
    .await
    .expect("Filesystem tests failed!");
}

async fn read_to_end<IO: embedded_io_async::Read>(io: &mut IO) -> Result<Vec<u8>, IO::Error> {
    let mut buf = Vec::new();
    loop {
        let mut tmp = [0; 512];
        match io.read(&mut tmp).await {
            Ok(0) => break,
            Ok(n) => buf.extend(&tmp[..n]),
            Err(e) => return Err(e),
        }
    }

    Ok(buf)
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    mpfs_hal::low_power_loop_forever()
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Info);
}
