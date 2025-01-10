use super::println;

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

        const RESET: &str = "\u{001B}[0m";
        const RED: &str = "\u{001B}[31m";
        const GREEN: &str = "\u{001B}[32m";
        const YELLOW: &str = "\u{001B}[33m";
        const BLUE: &str = "\u{001B}[34m";
        const CYAN: &str = "\u{001B}[35m";

        #[cfg(feature = "log_colors")]
        let color = match record.level() {
            log::Level::Error => RED,
            log::Level::Warn => YELLOW,
            log::Level::Info => GREEN,
            log::Level::Debug => BLUE,
            log::Level::Trace => CYAN,
        };
        #[cfg(feature = "log_colors")]
        let reset = RESET;

        #[cfg(not(feature = "log_colors"))]
        let color = "";
        #[cfg(not(feature = "log_colors"))]
        let reset = "";

        println!("{}{} - {}{}", color, record.level(), record.args(), reset);
    }

    fn flush(&self) {}
}
