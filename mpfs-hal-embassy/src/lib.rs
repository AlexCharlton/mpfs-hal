#![no_std]

extern crate alloc;

mod executor;
pub use executor::Executor;

pub mod time_driver;

pub use mpfs_hal_procmacros::{
    embassy_hart1_main, embassy_hart2_main, embassy_hart3_main, embassy_hart4_main,
};
pub use static_cell;

#[no_mangle]
fn __init_once_embassy() {
    unsafe {
        time_driver::init();
    }
}
