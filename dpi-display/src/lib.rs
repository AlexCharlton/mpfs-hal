#![no_std]

#[cfg(feature = "resolution-800x480")]
mod resolution {
    pub const WIDTH: usize = 800;
    pub const HEIGHT: usize = 480;
}

pub use resolution::*;
pub const BUFFER_SIZE: usize = WIDTH * HEIGHT * 3; // 3 bytes per pixel
pub const BUFFER_SEPARATION: usize = BUFFER_SIZE;

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

    pub struct DisplayBuffer<'a> {
        buffer: &'static mut [u8],
        is_buffer1: bool,
        buffer0_ready: &'a mut Buffer0Ready,
        buffer1_ready: &'a mut Buffer1Ready,
    }

    impl<'a> DisplayBuffer<'a> {
        pub fn as_slice(&mut self) -> &mut [u8] {
            self.buffer
        }

        pub fn set_pixel(&mut self, x: usize, y: usize, color: (u8, u8, u8)) {
            let index = (y * WIDTH + x) * 3;
            self.buffer[index] = color.0;
            self.buffer[index + 1] = color.1;
            self.buffer[index + 2] = color.2;
        }

        // Set a pixel at a specific pixel index
        //
        // I.e. `set_pixel_at_index(10 + 10 * WIDTH, (255, 0, 0))`
        // will set the pixel at (10, 10) to red
        pub fn set_pixel_at_index(&mut self, index: usize, color: (u8, u8, u8)) {
            let index = index * 3;
            self.buffer[index] = color.0;
            self.buffer[index + 1] = color.1;
            self.buffer[index + 2] = color.2;
        }
    }

    impl<'a> Drop for DisplayBuffer<'a> {
        fn drop(&mut self) {
            core::sync::atomic::fence(core::sync::atomic::Ordering::Release); // Ensure writes complete
            if self.is_buffer1 {
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

    pub struct Display {
        base_addr: *mut u8,
        use_buffer1_next: bool,
        buffer0_locked: Buffer0Locked,
        buffer1_locked: Buffer1Locked,
        buffer0_ready: Buffer0Ready,
        buffer1_ready: Buffer1Ready,
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
            }
        }
    }

    impl Display {
        fn _get_buffer(&mut self, upper: bool) -> DisplayBuffer {
            let offset = if upper { BUFFER_SEPARATION } else { 0 };
            let buffer =
                unsafe { slice::from_raw_parts_mut(self.base_addr.add(offset), BUFFER_SIZE) };
            DisplayBuffer {
                buffer,
                is_buffer1: upper,
                buffer0_ready: &mut self.buffer0_ready,
                buffer1_ready: &mut self.buffer1_ready,
            }
        }

        pub async fn get_buffer(&mut self) -> DisplayBuffer {
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
            self._get_buffer(use_buffer1_next)
        }
    }
}

pub use buffer::*;
