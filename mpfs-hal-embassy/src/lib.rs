#![no_std]
#![feature(impl_trait_in_assoc_type)]

mod executor;
pub use executor::Executor;

pub mod time_driver;

#[no_mangle]
fn __init_once() {
    unsafe {
        time_driver::init();
    }
}
