#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

use embedded_io_async::Read;
use mpfs_hal::uart::*;
use mpfs_hal::Peripheral;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    let mut rx = UartRx::new(UartRx0::take().unwrap(), UartConfig::default());
    println!("Hello, world!");
    let mut buf = [0; 128];
    loop {
        let n = rx.read(&mut buf).await;
        print!("Got: ");
        if let Ok(n) = n {
            for b in &buf[..n] {
                print!("{}", *b as char);
            }
            println!();
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    loop {}
}
