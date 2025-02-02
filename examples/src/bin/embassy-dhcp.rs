#![no_std]
#![no_main]

// Note that you need to have some kind of DHCP server running on the network
// for this to work.
// See other embassy-net examples (e.g. https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/ethernet_w5500_multisocket.rs) for how to create TCP connections.

use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources};
use rand::{rngs::SmallRng, RngCore, SeedableRng};
use static_cell::StaticCell;

#[macro_use]
extern crate mpfs_hal;
use mpfs_hal::ethernet::{EthernetDevice, MAC0};
use mpfs_hal::PeripheralRef;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(spawner: embassy_executor::Spawner) {
    println!("Initializing ethernet");
    let device = EthernetDevice::<MAC0>::take().unwrap();
    device.init([0x02, 0x01, 0x02, 0x03, 0x04, 0x05]);

    // Init network stack
    let mut rng = SmallRng::seed_from_u64(unsafe { mpfs_hal::pac::readmcycle() } as u64);
    let seed = rng.next_u64();
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(net_task(runner)).unwrap();

    log::info!("Waiting for DHCP...");
    // Why does this take so long?
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    log::info!("IP address: {:?}", local_addr);
}

#[embassy_executor::task]
async fn net_task(
    mut runner: embassy_net::Runner<'static, &'static mut EthernetDevice<MAC0>>,
) -> ! {
    runner.run().await
}

async fn wait_for_config(stack: Stack<'static>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config.clone();
        }
        yield_now().await;
    }
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Debug);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
