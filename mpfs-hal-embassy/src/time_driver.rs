use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use mpfs_hal::pac;

// Modelled off https://github.com/embassy-rs/embassy/blob/main/embassy-rp/src/time_driver.rs
// and https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples/blob/main/driver-examples/mss/mss-timer/mpfs-timer-example/src/application/hart1/u54_1.c

struct AlarmState {
    timestamp: Cell<u64>,
}
unsafe impl Send for AlarmState {}

const ALARM_COUNT: usize = 4;
const TIMER_VS_MTIME_RATIO: u64 =
    pac::LIBERO_SETTING_MSS_APB_AHB_CLK as u64 / pac::LIBERO_SETTING_MSS_RTC_TOGGLE_CLK as u64;

struct TimeDriver {
    queues: Mutex<CriticalSectionRawMutex, RefCell<[Queue; 4]>>,
    alarms: Mutex<CriticalSectionRawMutex, [AlarmState; 4]>,
    current_alarm: AtomicU8,
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimeDriver = TimeDriver {
    queues: Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new([const{Queue::new()}; 4])),
    alarms: Mutex::const_new(CriticalSectionRawMutex::new(), [const{AlarmState {
        timestamp: Cell::new(0),
    }}; ALARM_COUNT]),
    current_alarm: AtomicU8::new(0), // Index of the current alarm (i.e. hart_id - 1)
});

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        unsafe { pac::readmtime() }
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queues = self.queues.borrow(cs).borrow_mut();
            let queue = &mut queues[pac::hart_id() as usize - 1];

            #[cfg(feature = "debug_logs")]
            mpfs_hal::print_unguarded!("schedule wake for hart {} at {}\n", pac::hart_id(), at);

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next, pac::hart_id() as usize) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

impl TimeDriver {
    // Returns false if the alarm is already expired, otherwise true
    fn set_alarm(
        &self,
        cs: critical_section::CriticalSection,
        timestamp: u64,
        hart_id: usize,
    ) -> bool {
        let alarms = &self.alarms.borrow(cs);
        let alarm = &alarms[hart_id - 1];
        alarm.timestamp.set(timestamp);
        #[cfg(feature = "debug_logs")]
        mpfs_hal::print_unguarded!("Setting alarm for hart {} to {}\n", hart_id, timestamp,);

        let current_alarm = &alarms[self.current_alarm.load(Ordering::Acquire) as usize];
        if timestamp > current_alarm.timestamp.get() || timestamp == u64::MAX {
            return true; // We have another alarm that will trigger first
        }
        let now = self.now();
        if timestamp <= now {
            #[cfg(feature = "debug_logs")]
            mpfs_hal::print_unguarded!("Alarm already expired\n");
            alarm.timestamp.set(u64::MAX);
            return false; // Already expired
        }
        self.current_alarm
            .store(hart_id as u8 - 1, Ordering::Release);
        if timestamp == u64::MAX {
            #[cfg(feature = "debug_logs")]
            mpfs_hal::print_unguarded!("Alarm set to max\n");
            return true; // The alarm is "set" (but it will never trigger, since it's set to max)
        }
        #[cfg(feature = "debug_logs")]
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
            let mut queues = self.queues.borrow(cs).borrow_mut();

            // Dequeue the current alarm
            let alarms = self.alarms.borrow(cs);
            let current_alarm = self.current_alarm.load(Ordering::Acquire) as usize;
            let alarm = &alarms[current_alarm];
            alarm.timestamp.set(u64::MAX);
            let queue = &mut queues[current_alarm];
            let mut next = queue.next_expiration(self.now());
            unsafe {
                // Since it has work to do, wake up the hart corresponding to the current alarm
                pac::raise_soft_interrupt(current_alarm + 1);
            }
            while !self.set_alarm(cs, next, current_alarm + 1) {
                next = queue.next_expiration(self.now());
            }

            // Find the next alarm to set
            let mut pending_alarm: Option<usize> = None;
            for i in 0..ALARM_COUNT {
                let ts = alarms[i].timestamp.get();
                if ts != u64::MAX
                    && (pending_alarm.is_none()
                        || ts < alarms[pending_alarm.unwrap()].timestamp.get())
                {
                    pending_alarm = Some(i);
                }
            }

            // Set the next alarm, if there is one
            if let Some(pending_alarm) = pending_alarm {
                let alarm = &alarms[pending_alarm];
                let ts = alarm.timestamp.get();
                #[cfg(feature = "debug_logs")]
                mpfs_hal::print_unguarded!(
                    "Setting alarm {} from hart {} to {}\n",
                    pending_alarm,
                    pac::hart_id(),
                    ts
                );
                let now = self.now();
                let interval = if ts < now { 0 } else { ts - now };
                self._set_alarm(interval);
                self.current_alarm
                    .store(pending_alarm as u8, Ordering::Release);
                true
            } else {
                unsafe {
                    pac::MSS_TIM64_stop(pac::TIMER_LO);
                }
                false
            }
        });
        unsafe {
            pac::MSS_TIM64_clear_irq(pac::TIMER_LO);
        }
        ret
    }
}

/// Safety: must be called exactly once at bootup
pub(crate) unsafe fn init() {
    critical_section::with(|cs| {
        let alarms = DRIVER.alarms.borrow(cs);
        for a in alarms {
            a.timestamp.set(u64::MAX);
        }
    });

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
    #[cfg(feature = "debug_logs")]
    mpfs_hal::print_unguarded!("Hart {} timer! at {}\n", pac::hart_id(), DRIVER.now());

    let pending = DRIVER.trigger_alarm();

    #[cfg(feature = "debug_logs")]
    mpfs_hal::print_unguarded!("returning from timer\n");

    return if pending {
        pac::EXT_IRQ_KEEP_ENABLED
    } else {
        pac::EXT_IRQ_DISABLE
    } as u8;
}
