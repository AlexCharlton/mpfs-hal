use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use mpfs_hal::pac;

// Modelled off https://github.com/embassy-rs/embassy/blob/main/embassy-rp/src/time_driver.rs
// and https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples/blob/main/driver-examples/mss/mss-timer/mpfs-timer-example/src/application/hart1/u54_1.c

const TIMER_VS_MTIME_RATIO: u64 =
    pac::LIBERO_SETTING_MSS_APB_AHB_CLK as u64 / pac::LIBERO_SETTING_MSS_RTC_TOGGLE_CLK as u64;

struct TimeDriver {
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimeDriver = TimeDriver {
    queue: Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(Queue::new())),
});

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        unsafe { pac::readmtime() }
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            #[cfg(feature = "debug-logs")]
            mpfs_hal::print_unguarded!("schedule wake for hart {} at {}\n", pac::hart_id(), at);

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

impl TimeDriver {
    // Returns false if the alarm is already expired, otherwise true
    fn set_alarm(&self, timestamp: u64) -> bool {
        #[cfg(feature = "debug-logs")]
        mpfs_hal::print_unguarded!("Setting alarm for hart {} to {}\n", hart_id, timestamp,);

        let now = self.now();
        if timestamp <= now {
            #[cfg(feature = "debug-logs")]
            mpfs_hal::print_unguarded!("Alarm already expired\n");
            return false; // Already expired
        }
        if timestamp == u64::MAX {
            #[cfg(feature = "debug-logs")]
            mpfs_hal::print_unguarded!("Alarm set to max\n");
            return true; // The alarm is "set" (but it will never trigger, since it's set to max)
        }
        #[cfg(feature = "debug-logs")]
        mpfs_hal::print_unguarded!("Setting alarm\n");

        let diff = timestamp - now;
        self._set_alarm(diff);
        true
    }

    fn _set_alarm(&self, interval: u64) {
        let counter = interval.saturating_mul(TIMER_VS_MTIME_RATIO);
        let load_value_u = (counter >> 32) as u32;
        let load_value_l = counter as u32;
        unsafe {
            pac::MSS_TIM64_load_immediate(pac::TIMER_LO, load_value_u, load_value_l);
            pac::MSS_TIM64_start(pac::TIMER_LO);
            pac::MSS_TIM64_enable_irq(pac::TIMER_LO);
        }
    }

    // Returns true if there is a pending alarm
    fn trigger_alarm(&self) -> bool {
        let ret = critical_section::with(|cs| {
            // Dequeue the current alarm
            let mut queue = self.queue.borrow(cs).borrow_mut();
            let mut next = queue.next_expiration(self.now());
            while !self.set_alarm(next) {
                next = queue.next_expiration(self.now());
            }

            // We have a pending alarm if the next expiration is not max
            next != u64::MAX
        });
        unsafe {
            if !ret {
                pac::MSS_TIM64_stop(pac::TIMER_LO);
            }
            pac::MSS_TIM64_clear_irq(pac::TIMER_LO);
        }
        ret
    }
}

/// Safety: must be called exactly once at bootup
pub(crate) fn init() {
    unsafe {
        pac::mss_config_clk_rst(
            pac::mss_peripherals__MSS_PERIPH_TIMER,
            pac::MPFS_HAL_FIRST_HART as u8,
            pac::PERIPH_RESET_STATE__PERIPHERAL_ON,
        );
        pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_TIMER1_INT_OFFSET, 2);
        pac::PLIC_SetPriority(pac::PLIC_IRQn_Type_PLIC_TIMER2_INT_OFFSET, 2);
        pac::reset_mtime();
        pac::MSS_TIM64_init(pac::TIMER_LO, pac::__mss_timer_mode_MSS_TIMER_ONE_SHOT_MODE);
    }
}

#[no_mangle]
extern "C" fn PLIC_timer1_IRQHandler() -> u8 {
    #[cfg(feature = "debug-logs")]
    mpfs_hal::print_unguarded!("Hart {} timer! at {}\n", pac::hart_id(), DRIVER.now());

    let pending = DRIVER.trigger_alarm();

    #[cfg(feature = "debug-logs")]
    mpfs_hal::print_unguarded!("returning from timer\n");

    return if pending {
        pac::EXT_IRQ_KEEP_ENABLED
    } else {
        pac::EXT_IRQ_DISABLE
    } as u8;
}
