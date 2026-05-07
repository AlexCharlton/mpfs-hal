#[allow(unused)]
#[cfg(feature = "log")]
macro_rules! info {
    ($($arg:tt)*) => {
        ::log::info!($($arg)*);
    };
}
#[allow(unused)]
#[cfg(not(feature = "log"))]
macro_rules! info {
    ($($arg:tt)*) => {};
}

#[cfg(feature = "log")]
macro_rules! debug {
    ($($arg:tt)*) => {
        ::log::debug!($($arg)*);
    };
}
#[cfg(not(feature = "log"))]
macro_rules! debug {
    ($($arg:tt)*) => {};
}

#[cfg(feature = "log")]
macro_rules! warn {
    ($($arg:tt)*) => {
        ::log::warn!($($arg)*);
    };
}
#[cfg(not(feature = "log"))]
macro_rules! warn {
    ($($arg:tt)*) => {};
}

#[cfg(feature = "log")]
macro_rules! error {
    ($($arg:tt)*) => {
        ::log::error!($($arg)*);
    };
}
#[cfg(not(feature = "log"))]
macro_rules! error {
    ($($arg:tt)*) => {};
}

#[cfg(feature = "log")]
macro_rules! trace {
    ($($arg:tt)*) => {
        ::log::trace!($($arg)*);
    };
}
#[cfg(not(feature = "log"))]
macro_rules! trace {
    ($($arg:tt)*) => {};
}
