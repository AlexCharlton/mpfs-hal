pub trait Peripheral {
    fn take() -> Option<Self>
    where
        Self: Sized;
    unsafe fn steal() -> Self;
}

pub trait PeripheralRef {
    fn take() -> Option<&'static mut Self>
    where
        Self: Sized;
    unsafe fn steal() -> &'static mut Self;
}
