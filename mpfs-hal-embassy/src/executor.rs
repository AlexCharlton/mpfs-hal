use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::{raw, Spawner};

use mpfs_hal::pac;

static SIGNAL_WORK_THREAD_MODE: [AtomicBool; pac::MPFS_HAL_LAST_HART as usize] =
    [const { AtomicBool::new(false) }; pac::MPFS_HAL_LAST_HART as usize];

#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    #[cfg(feature = "debug_logs")]
    mpfs_hal::println_unguarded!("hart {} has work pending\n", context as usize + 1);
    SIGNAL_WORK_THREAD_MODE[context as usize].store(true, Ordering::SeqCst);
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
                self.inner.poll();
                let ctx = pac::hart_id() - 1;
                let mut do_wfi = true;
                critical_section::with(|_| {
                    // If there is work to do, loop back to polling
                    // TODO can we relax this?
                    if SIGNAL_WORK_THREAD_MODE[ctx].load(Ordering::SeqCst) {
                        #[cfg(feature = "debug_logs")]
                        mpfs_hal::println_unguarded!("hart {} has work to do\n", ctx + 1);
                        SIGNAL_WORK_THREAD_MODE[ctx].store(false, Ordering::SeqCst);
                        do_wfi = false;
                    }
                });
                // If not, wait for interrupt
                // This is not in the critical section, since we want to release the critical-section lock
                if do_wfi {
                    #[cfg(feature = "debug_logs")]
                    mpfs_hal::println_unguarded!("hart {} going to wfi\n", ctx + 1);
                    core::arch::asm!("wfi");
                    #[cfg(feature = "debug_logs")]
                    mpfs_hal::println_unguarded!("hart {} wfi\n", ctx + 1);
                }
                // if an interrupt occurred while waiting, it will be serviced here
            }
        }
    }
}
