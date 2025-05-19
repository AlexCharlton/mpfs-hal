#![no_std]

#[cfg(feature = "resolution-800x480")]
mod resolution {
    pub const WIDTH: usize = 800;
    pub const HEIGHT: usize = 480;
}

pub use resolution::*;
