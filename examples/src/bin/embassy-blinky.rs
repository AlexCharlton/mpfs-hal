#![no_std]
#![no_main]

extern crate alloc;

use embassy_time::Timer;
use mpfs_hal::pac;

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    unsafe {
        pac::MSS_GPIO_init(pac::GPIO2_LO);
        pac::MSS_GPIO_config(
            pac::GPIO2_LO,
            pac::mss_gpio_id_MSS_GPIO_0,
            pac::MSS_GPIO_OUTPUT_MODE,
        );
    }

    loop {
        Timer::after_millis(500).await;
        unsafe {
            pac::MSS_GPIO_set_output(pac::GPIO2_LO, pac::mss_gpio_id_MSS_GPIO_0, 1);
        }
        Timer::after_millis(500).await;
        unsafe {
            pac::MSS_GPIO_set_output(pac::GPIO2_LO, pac::mss_gpio_id_MSS_GPIO_0, 0);
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::uart_print_panic(info);
    loop {}
}
