use super::println;
use alloc::string::String;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

extern crate alloc;

// Structure to hold log messages
struct LogMessage {
    level: log::Level,
    args: String,
    timestamp: u64,
}

// Static channel for log messages
static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogMessage, 1024> = Channel::new();
static mut LOGGER: MpfsLogger = MpfsLogger {
    level: log::LevelFilter::Error,
};

/// Initialize the logger with the given maximum log level.
pub fn init_logger(level: log::LevelFilter) {
    unsafe {
        LOGGER.level = level;
        #[allow(static_mut_refs)]
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level_racy(level);
    }
}

pub async fn log_task() -> ! {
    let receiver = LOG_CHANNEL.receiver();
    loop {
        // Try to receive a message from the channel
        if let Ok(message) = receiver.try_receive() {
            const RESET: &str = "\u{001B}[0m";
            const RED: &str = "\u{001B}[31m";
            const GREEN: &str = "\u{001B}[32m";
            const YELLOW: &str = "\u{001B}[33m";
            const BLUE: &str = "\u{001B}[34m";
            const CYAN: &str = "\u{001B}[35m";

            #[cfg(feature = "log-colors")]
            let color = match message.level {
                log::Level::Error => RED,
                log::Level::Warn => YELLOW,
                log::Level::Info => GREEN,
                log::Level::Debug => BLUE,
                log::Level::Trace => CYAN,
            };
            #[cfg(feature = "log-colors")]
            let reset = RESET;

            #[cfg(not(feature = "log-colors"))]
            let color = "";
            #[cfg(not(feature = "log-colors"))]
            let reset = "";

            println!(
                "{}{} [{:?}] - {}{}",
                color, message.level, message.timestamp, message.args, reset
            );
        }
        yield_now().await;
    }
}

struct MpfsLogger {
    level: log::LevelFilter,
}

impl log::Log for MpfsLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= self.level
    }

    #[allow(unused)]
    fn log(&self, record: &log::Record) {
        if !self.enabled(&record.metadata()) {
            return;
        }

        let mut args = String::new();
        let _ = core::fmt::write(&mut args, *record.args());

        // Create the log message with timestamp
        let message = LogMessage {
            level: record.level(),
            args,
            timestamp: unsafe { crate::pac::readmtime() },
        };

        let _ = LOG_CHANNEL.try_send(message);
    }

    fn flush(&self) {}
}

//----------------------------------------------------------
// Yield now
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

fn yield_now() -> impl Future<Output = ()> {
    YieldNowFuture { yielded: false }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct YieldNowFuture {
    yielded: bool,
}

impl Future for YieldNowFuture {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.yielded {
            Poll::Ready(())
        } else {
            self.yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}
