#![no_std]

#[cfg(feature = "embedded-graphics")]
mod embedded_graphics;
#[cfg(feature = "embedded-graphics")]
pub use embedded_graphics::*;

#[cfg(feature = "resolution-800x480")]
mod resolution {
    pub const WIDTH: usize = 800;
    pub const HEIGHT: usize = 480;
}

pub use resolution::*;
pub const BUFFER_SIZE: usize = WIDTH * HEIGHT; // In pixels
pub const BUFFER_SIZE_BYTES: usize = BUFFER_SIZE * 3; // In bytes; 3 bytes per pixel
pub const BUFFER_SEPARATION: usize = BUFFER_SIZE_BYTES; // In bytes

pub fn init() {
    mpfs_hal::gpio::init();
    // Zero the buffer memory
    let buffer = unsafe {
        core::slice::from_raw_parts_mut(mpfs_hal::pac::heap_end() as *mut u8, BUFFER_SIZE_BYTES * 2)
    };
    buffer.fill(0);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pixel {
    r: u8,
    g: u8,
    b: u8,
}

impl Pixel {
    pub const BLACK: Self = Self { r: 0, g: 0, b: 0 };
    pub const WHITE: Self = Self {
        r: 255,
        g: 255,
        b: 255,
    };
    pub const RED: Self = Self { r: 255, g: 0, b: 0 };
    pub const GREEN: Self = Self { r: 0, g: 255, b: 0 };
    pub const BLUE: Self = Self { r: 0, g: 0, b: 255 };
    pub const YELLOW: Self = Self {
        r: 255,
        g: 255,
        b: 0,
    };
    pub const CYAN: Self = Self {
        r: 0,
        g: 255,
        b: 255,
    };
    pub const MAGENTA: Self = Self {
        r: 255,
        g: 0,
        b: 255,
    };

    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

// Inputs/outputs for display synchronization
mod io {
    use mpfs_hal::{
        gpio::{GpioPeripheral, GPIO1_10_OR_GPIO2_24_INT, GPIO1_11_OR_GPIO2_25_INT},
        impl_gpio_pin, impl_input_peripheral, impl_output_peripheral, pac, Peripheral,
    };

    impl_gpio_pin!(BUFFER0_READY, GpioPeripheral::Mss(pac::GPIO2_LO), 26, NONE);
    impl_output_peripheral!(Buffer0Ready, BUFFER0_READY);

    impl_gpio_pin!(BUFFER1_READY, GpioPeripheral::Mss(pac::GPIO2_LO), 27, NONE);
    impl_output_peripheral!(Buffer1Ready, BUFFER1_READY);

    impl_gpio_pin!(
        BUFFER0_LOCKED,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        24,
        GPIO1_10_OR_GPIO2_24_INT
    );
    impl_input_peripheral!(Buffer0Locked, BUFFER0_LOCKED);

    impl_gpio_pin!(
        BUFFER1_LOCKED,
        GpioPeripheral::Mss(pac::GPIO2_LO),
        25,
        GPIO1_11_OR_GPIO2_25_INT
    );
    impl_input_peripheral!(Buffer1Locked, BUFFER1_LOCKED);
}

mod buffer {
    use super::*;
    use core::slice;
    use io::*;

    use embedded_hal::digital::OutputPin;
    use embedded_hal_async::digital::Wait;

    pub struct DisplayBuffer {
        buffer: &'static mut [Pixel],
        is_buffer1: bool,
    }

    impl DisplayBuffer {
        pub fn as_slice(&mut self) -> &mut [Pixel] {
            self.buffer
        }

        pub fn as_u8_slice(&mut self) -> &mut [u8] {
            unsafe { slice::from_raw_parts_mut(self.buffer as *mut _ as *mut u8, BUFFER_SIZE * 3) }
        }

        pub fn set_pixel(&mut self, x: usize, y: usize, color: Pixel) {
            self.buffer[y * WIDTH + x] = color;
        }

        // Set a pixel at a specific pixel index
        //
        // I.e. `set_pixel_at_index(10 + 10 * WIDTH, (255, 0, 0))`
        // will set the pixel at (10, 10) to red
        pub fn set_pixel_at_index(&mut self, index: usize, color: Pixel) {
            self.buffer[index] = color;
        }
    }

    pub struct Display {
        base_addr: *mut u8,
        use_buffer1_next: bool,
        buffer0_locked: Buffer0Locked,
        buffer1_locked: Buffer1Locked,
        buffer0_ready: Buffer0Ready,
        buffer1_ready: Buffer1Ready,
        display_buffer: Option<DisplayBuffer>,
    }

    static mut DISPLAY_TAKEN: bool = false;

    impl mpfs_hal::Peripheral for Display {
        fn take() -> Option<Self> {
            critical_section::with(|_| unsafe {
                if DISPLAY_TAKEN {
                    None
                } else {
                    let buffer0_locked = Buffer0Locked::take();
                    let buffer1_locked = Buffer1Locked::take();
                    let buffer0_ready = Buffer0Ready::take();
                    let buffer1_ready = Buffer1Ready::take();

                    if let (
                        Some(buffer0_locked),
                        Some(buffer1_locked),
                        Some(buffer0_ready),
                        Some(buffer1_ready),
                    ) = (buffer0_locked, buffer1_locked, buffer0_ready, buffer1_ready)
                    {
                        DISPLAY_TAKEN = true;
                        Some(Self {
                            base_addr: mpfs_hal::pac::heap_end() as *mut u8,
                            use_buffer1_next: false,
                            buffer0_locked,
                            buffer1_locked,
                            buffer0_ready,
                            buffer1_ready,
                            display_buffer: None,
                        })
                    } else {
                        None
                    }
                }
            })
        }

        unsafe fn steal() -> Self {
            Self {
                base_addr: mpfs_hal::pac::heap_end() as *mut u8,
                use_buffer1_next: false,
                buffer0_locked: Buffer0Locked::steal(),
                buffer1_locked: Buffer1Locked::steal(),
                buffer0_ready: Buffer0Ready::steal(),
                buffer1_ready: Buffer1Ready::steal(),
                display_buffer: None,
            }
        }
    }

    impl Display {
        fn _get_buffer(&mut self, upper: bool) -> DisplayBuffer {
            let offset = if upper { BUFFER_SEPARATION } else { 0 };
            let buffer = unsafe {
                slice::from_raw_parts_mut(self.base_addr.add(offset) as *mut Pixel, BUFFER_SIZE)
            };
            DisplayBuffer {
                buffer,
                is_buffer1: upper,
            }
        }

        pub async fn get_buffer(&mut self) -> &mut DisplayBuffer {
            if self.display_buffer.is_none() {
                let use_buffer1_next = self.use_buffer1_next;
                self.use_buffer1_next = !use_buffer1_next;
                if use_buffer1_next {
                    log::trace!("Waiting for display buffer 1 to be unlocked");
                    let _ = self.buffer1_locked.wait_for_low().await;
                } else {
                    log::trace!("Waiting for display buffer 0 to be unlocked");
                    let _ = self.buffer0_locked.wait_for_low().await;
                }
                log::trace!("Got display buffer");
                self.display_buffer = Some(self._get_buffer(use_buffer1_next));
            }
            self.display_buffer.as_mut().unwrap()
        }

        pub fn maybe_get_buffer(&mut self) -> Option<&mut DisplayBuffer> {
            self.display_buffer.as_mut()
        }

        pub fn flush(&mut self) {
            if let Some(display_buffer) = self.display_buffer.take() {
                core::sync::atomic::fence(core::sync::atomic::Ordering::Release); // Ensure writes complete
                if display_buffer.is_buffer1 {
                    let _ = self.buffer0_ready.set_low().unwrap();
                    log::trace!("Display buffer 0 marked not ready");
                    self.buffer1_ready.set_high().unwrap();
                    log::trace!("Display buffer 1 marked ready");
                } else {
                    let _ = self.buffer1_ready.set_low().unwrap();
                    log::trace!("Display buffer 1 marked not ready");
                    self.buffer0_ready.set_high().unwrap();
                    log::trace!("Display buffer 0 marked ready");
                }
            }
        }
    }
}

pub use buffer::*;
