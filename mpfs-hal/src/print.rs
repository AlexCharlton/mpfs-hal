use crate::{uart::*, Mutex, Peripheral};
use core::panic::PanicInfo;

use embedded_io::Write;

static mut UART: Option<Uart<UART0>> = None;
static MUTEX: Mutex = Mutex::new();

pub(crate) fn init_print() {
    unsafe { UART = Some(Uart::new(UART0::take().unwrap(), UartConfig::default())) }
}

pub struct Printer;

impl Printer {
    // We need to hold onto a lock _longer_ than just what `write_str` needs
    // because `writeln` will call `write_str` multiple times.
    pub fn with_lock<F>(f: F)
    where
        F: FnOnce(),
    {
        let lock_token = MUTEX.lock();
        f();
        MUTEX.release(lock_token);
    }
}

impl core::fmt::Write for Printer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let uart = unsafe { UART.as_mut().unwrap() };
        uart.write(s.as_bytes()).unwrap();
        Ok(())
    }
}

#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {{
        {
            use core::fmt::Write;
            $crate::Printer::with_lock(|| {
                writeln!($crate::Printer, $($arg)*).ok();
            });
        }
    }};
}

#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        {
            use core::fmt::Write;
            $crate::Printer::with_lock(|| {
                write!($crate::Printer, $($arg)*).ok();
            });
        }
    }};
}

pub fn print_panic(panic: &PanicInfo<'_>) {
    if let Some(location) = panic.location() {
        println!(
            "PANIC at {}:{}: {}",
            location.file(),
            location.line(),
            panic.message()
        );
    }
}

pub struct UnguardedPrinter;

impl core::fmt::Write for UnguardedPrinter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        unsafe {
            // If the UART is not initialized, do nothing
            if UART.is_none() {
                return Ok(());
            }
            super::pac::MSS_UART_polled_tx(UART0::steal().address(), s.as_ptr(), s.len() as u32);
        }
        Ok(())
    }
}

#[macro_export]
macro_rules! println_unguarded {
    ($($arg:tt)*) => {{
        {
            use core::fmt::Write;
            writeln!($crate::UnguardedPrinter, $($arg)*).ok();
        }
    }};
}

#[macro_export]
macro_rules! print_unguarded {
    ($($arg:tt)*) => {{
        {
            use core::fmt::Write;
            write!($crate::UnguardedPrinter, $($arg)*).ok();
        }
    }};
}
