#![no_std]
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

mod executor;
pub use executor::Executor;

pub mod time_driver;

pub use mpfs_hal_procmacros::*;
pub use static_cell;

#[no_mangle]
fn __init_once() {
    unsafe {
        time_driver::init();
    }
}
