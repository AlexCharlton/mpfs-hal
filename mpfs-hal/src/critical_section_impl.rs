use core::sync::atomic::{compiler_fence, AtomicUsize, Ordering};
use critical_section::RawRestoreState;

use super::pac::{hart_id, MIP_MSIP};

pub struct MPFSCriticalSection;

const LOCK_UNOWNED: usize = 0;

// Stores which hart (core) owns the lock: 0 = unowned, 1-4 = hart ID
static LOCK_OWNER: AtomicUsize = AtomicUsize::new(LOCK_UNOWNED);

#[repr(u8)]
enum RestoreState {
    ReleaseLockLeaveInterruptsDisabled,
    ReleaseLockRestoreInterrupts,
    KeepLock,
}

unsafe impl critical_section::Impl for MPFSCriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        let hart_id: usize = hart_id();
        // Check if we already own the lock
        if LOCK_OWNER.load(Ordering::Acquire) == hart_id {
            // We already own the lock, so no need to lock again
            // When we release this particular "lock", we won't do anything
            // The initial lock-taker is responsible for actually releasing the lock
            return RestoreState::KeepLock as RawRestoreState;
        }

        // First check if interrupts are enabled
        let mut status: usize;
        core::arch::asm!(
            "csrr {}, mstatus",
            out(reg) status
        );
        // When entering from an interrupt handler, interrupts are already disabled
        // So we only want to restore them if they were enabled when we entered
        let was_enabled = (status & MIP_MSIP) != 0;

        loop {
            // Disable interrupts
            core::arch::asm!(
                "csrci mstatus, {}",
                const MIP_MSIP,
                options(nomem, nostack)
            );
            compiler_fence(Ordering::SeqCst);

            match LOCK_OWNER.compare_exchange(
                LOCK_UNOWNED,
                hart_id,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => break,
                Err(_) => {
                    // Re-enable interrupts if they were enabled before
                    if was_enabled {
                        core::arch::asm!(
                            "csrsi mstatus, {}",
                            const MIP_MSIP,
                            options(nomem, nostack)
                        );
                    }
                }
            }
        }

        #[cfg(feature = "debug_logs")]
        {
            crate::print_unguarded!("hart {} acquired", hart_id);
            if was_enabled {
                crate::println_unguarded!(" (restore interrupts: true)\n");
            } else {
                crate::println_unguarded!(" (restore: false)\n");
            }
        }

        if was_enabled {
            // We need to restore interrupts when we release the lock
            RestoreState::ReleaseLockRestoreInterrupts as RawRestoreState
        } else {
            // Interrupts were disabled when we entered, so we don't restore them when releasing
            RestoreState::ReleaseLockLeaveInterruptsDisabled as RawRestoreState
        }
    }

    unsafe fn release(restore_state: RawRestoreState) {
        let state = restore_state as u8;
        if state == RestoreState::KeepLock as u8 {
            return;
        }

        #[cfg(feature = "debug_logs")]
        {
            crate::println_unguarded!("hart {} releasing\n", hart_id());
        }

        // Release the lock
        LOCK_OWNER.store(LOCK_UNOWNED, Ordering::Release);

        if state == RestoreState::ReleaseLockRestoreInterrupts as u8 {
            compiler_fence(Ordering::SeqCst);
            // Re-enable interrupts
            core::arch::asm!("csrsi mstatus, {}", const MIP_MSIP, options(nomem, nostack));
        }
    }
}
