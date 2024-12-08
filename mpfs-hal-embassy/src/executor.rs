use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_executor::{raw, Spawner};

#[cfg(feature = "debug_logs")]
use mpfs_hal::uart_puts;

use mpfs_pac as sys;

static SIGNAL_WORK_THREAD_MODE: [AtomicBool; sys::MPFS_HAL_LAST_HART as usize] =
    [const { AtomicBool::new(false) }; sys::MPFS_HAL_LAST_HART as usize];

#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    #[cfg(feature = "debug_logs")]
    {
        let msg = alloc::format!("hart {} has work pending\n\0", context as usize + 1);
        uart_puts(msg.as_ptr());
    }
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
            inner: raw::Executor::new((sys::hart_id() - 1) as *mut ()),
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
        unsafe {
            self.inner.initialize();
        }

        init(self.inner.spawner());

        loop {
            unsafe {
                self.inner.poll();
                let ctx = sys::hart_id() - 1;
                let mut do_wfi = true;
                critical_section::with(|_| {
                    // If there is work to do, loop back to polling
                    // TODO can we relax this?
                    if SIGNAL_WORK_THREAD_MODE[ctx].load(Ordering::SeqCst) {
                        #[cfg(feature = "debug_logs")]
                        {
                            let msg = alloc::format!("hart {} has work to do\n\0", ctx + 1);
                            uart_puts(msg.as_ptr());
                        }
                        SIGNAL_WORK_THREAD_MODE[ctx].store(false, Ordering::SeqCst);
                        do_wfi = false;
                    }
                });
                // If not, wait for interrupt
                // This is not in the critical section, since we want to release the critical-section lock
                if do_wfi {
                    #[cfg(feature = "debug_logs")]
                    {
                        let msg = alloc::format!("hart {} going to wfi\n\0", ctx + 1,);
                        uart_puts(msg.as_ptr());
                    }
                    core::arch::asm!("wfi");
                    #[cfg(feature = "debug_logs")]
                    {
                        let msg = alloc::format!("hart {} wfi\n\0", ctx + 1);
                        uart_puts(msg.as_ptr());
                    }
                }
                // if an interrupt occurred while waiting, it will be serviced here
            }
        }
    }
}
