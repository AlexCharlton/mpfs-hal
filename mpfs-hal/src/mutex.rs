use core::sync::atomic::{AtomicUsize, Ordering};
// A Mutex that does not protect against interrupts, but does protect against other cores
const LOCK_UNOWNED: usize = 0;

pub struct Mutex {
    owner: AtomicUsize,
}

pub struct LockToken(bool);

impl Mutex {
    pub const fn new() -> Self {
        Self {
            owner: AtomicUsize::new(LOCK_UNOWNED),
        }
    }

    pub fn lock(&self) -> LockToken {
        let hart_id = crate::hart_id();
        if self.owner.load(Ordering::Acquire) == hart_id {
            return LockToken(false);
        }
        loop {
            if self
                .owner
                .compare_exchange(LOCK_UNOWNED, hart_id, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                break;
            }
        }
        LockToken(true)
    }
    pub fn release(&self, lock_token: LockToken) {
        if lock_token.0 {
            self.owner.store(LOCK_UNOWNED, Ordering::Release);
        }
    }
}
