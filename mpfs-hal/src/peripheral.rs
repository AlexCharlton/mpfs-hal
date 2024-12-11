pub trait Peripheral {
    fn take() -> Option<Self>
    where
        Self: Sized;
    unsafe fn steal() -> Self;
}
