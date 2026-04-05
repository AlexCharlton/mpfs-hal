use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};
use embassy_executor::{Spawner, raw};

use mpfs_hal::pac;

static SIGNAL_WORK_THREAD_MODE: [AtomicBool; pac::MPFS_HAL_LAST_HART as usize] =
    [const { AtomicBool::new(false) }; pac::MPFS_HAL_LAST_HART as usize];

#[unsafe(export_name = "__pender")]
fn __pender(context: *mut ()) {
    #[cfg(feature = "debug-logs")]
    mpfs_hal::println_unguarded!("hart {} has work pending\n", context as usize + 1);
    SIGNAL_WORK_THREAD_MODE[context as usize].store(true, Ordering::SeqCst);
    unsafe {
        pac::raise_soft_interrupt(context as usize + 1);
    }
}

pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    /// Create a new Executor.
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new((pac::hart_id() - 1) as *mut ()),
            not_send: PhantomData,
        }
    }

    /// Run the executor.
    ///
    /// The `init` closure is called with a [`Spawner`] that spawns tasks on
    /// this executor. Use it to spawn the initial task(s). After `init` returns,
    /// the executor starts running the tasks.
    ///
    /// To spawn more tasks later, you may keep copies of the [`Spawner`] (it is `Copy`),
    /// for example by passing it as an argument to the initial tasks.
    ///
    /// This function requires `&'static mut self`. This means you have to store the
    /// Executor instance in a place where it'll live forever and grants you mutable
    /// access. There's a few ways to do this:
    ///
    /// - a [StaticCell](https://docs.rs/static_cell/latest/static_cell/) (safe)
    /// - a `static mut` (unsafe)
    /// - a local variable in a function you know never returns (like `fn main() -> !`), upgrading its lifetime with `transmute`. (unsafe)
    ///
    /// This function never returns.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            unsafe {
                // Perform any pending work
                self.inner.poll();

                // Disable interrupts
                core::arch::asm!(
                    "csrci mstatus, {}",
                    const pac::MIP_MSIP,
                    options(nomem, nostack)
                );
                compiler_fence(Ordering::SeqCst);

                let ctx = pac::hart_id() - 1;
                // If there is work to do, skip going into WFI and loop back to polling
                if !SIGNAL_WORK_THREAD_MODE[ctx].swap(false, Ordering::SeqCst) {
                    // Since we don't have any work to do, wait for an interrupt
                    #[cfg(feature = "debug-logs")]
                    mpfs_hal::println_unguarded!("hart {} wfi\n", ctx + 1);
                    core::arch::asm!("wfi");

                    #[cfg(feature = "debug-logs")]
                    mpfs_hal::println_unguarded!("hart {} waking from wfi\n", ctx + 1);
                }
                // Re-enable interrupts
                compiler_fence(Ordering::SeqCst);
                core::arch::asm!("csrsi mstatus, {}", const pac::MIP_MSIP, options(nomem, nostack));
            }
        }
    }
}
