use core::cell::Cell;
use core::sync::atomic::{AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::{AlarmHandle, Driver};

use mpfs_hal::pac;

// Modelled off https://github.com/embassy-rs/embassy/blob/main/embassy-rp/src/time_driver.rs
// and https://github.com/polarfire-soc/polarfire-soc-bare-metal-examples/blob/main/driver-examples/mss/mss-timer/mpfs-timer-example/src/application/hart1/u54_1.c

struct AlarmState {
    timestamp: Cell<u64>,
    hart: Cell<usize>,
    callback: Cell<Option<(fn(*mut ()), *mut ())>>,
}
unsafe impl Send for AlarmState {}

const ALARM_COUNT: usize = 4;
const TIMER_VS_MTIME_RATIO: u64 =
    pac::LIBERO_SETTING_MSS_APB_AHB_CLK as u64 / pac::LIBERO_SETTING_MSS_RTC_TOGGLE_CLK as u64;

struct TimeDriver {
    alarms: Mutex<CriticalSectionRawMutex, [AlarmState; ALARM_COUNT]>,
    current_alarm: AtomicU8,
    next_alarm: AtomicU8,
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimeDriver = TimeDriver {
    alarms: Mutex::const_new(CriticalSectionRawMutex::new(), [const{AlarmState {
        timestamp: Cell::new(0),
        hart: Cell::new(0),
        callback: Cell::new(None),
    }}; ALARM_COUNT]),
    current_alarm: AtomicU8::new(0),
    next_alarm: AtomicU8::new(0),
});

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        unsafe { pac::readmtime() }
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        let id = self
            .next_alarm
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
                if x < ALARM_COUNT as u8 {
                    Some(x + 1)
                } else {
                    None
                }
            });

        match id {
            Ok(id) => {
                critical_section::with(|cs| {
                    let alarms = self.alarms.borrow(cs);
                    alarms[id as usize].hart.set(pac::hart_id());
                });
                Some(AlarmHandle::new(id))
            }
            Err(_) => None,
        }
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarms = &self.alarms.borrow(cs);
            let alarm = &alarms[n];
            alarm.timestamp.set(timestamp);
            #[cfg(feature = "debug_logs")]
            mpfs_hal::print_unguarded!(
                "Setting alarm {} for hart {} (alarm hart {}) to {}",
                n,
                pac::hart_id(),
                alarm.hart.get(),
                timestamp
            );

            let current_alarm = &alarms[self.current_alarm.load(Ordering::Acquire) as usize];
            if timestamp > current_alarm.timestamp.get() || timestamp == u64::MAX {
                return true; // We have another alarm that will trigger first
            }
            let now = self.now();
            if timestamp <= now {
                alarm.timestamp.set(u64::MAX);
                return false; // Already expired
            }
            #[cfg(feature = "debug_logs")]
            mpfs_hal::print_unguarded!("Setting alarm\n");

            let diff = timestamp - now;
            self._set_alarm(diff, alarm.hart.get());
            self.current_alarm.store(n as u8, Ordering::Release);
            true
        })
    }
}

impl TimeDriver {
    fn _set_alarm(&self, interval: u64, hart_id: usize) {
        let counter = interval * TIMER_VS_MTIME_RATIO;
        let load_value_u = (counter >> 32) as u32;
        let load_value_l = counter as u32;
        unsafe {
            pac::MSS_TIM64_load_immediate(pac::TIMER_LO, load_value_u, load_value_l);
            pac::MSS_TIM64_start(pac::TIMER_LO);
            pac::MSS_TIM64_enable_irq_for_hart(pac::TIMER_LO, hart_id as u64);
        }
    }

    // Returns true if there is a pending alarm
    fn trigger_alarm(&self) -> bool {
        let ret = critical_section::with(|cs| {
            let now = self.now();
            let alarms = self.alarms.borrow(cs);
            let alarm = &alarms[self.current_alarm.load(Ordering::Acquire) as usize];
            alarm.timestamp.set(u64::MAX);
            if let Some((f, ctx)) = alarm.callback.get() {
                f(ctx);
            }
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

            if let Some(pending_alarm) = pending_alarm {
                let alarm = &alarms[pending_alarm];
                let ts = alarm.timestamp.get();
                #[cfg(feature = "debug_logs")]
                mpfs_hal::print_unguarded!(
                    "Setting alarm {} from hart {} (alarm hart {}) to {}\n",
                    pending_alarm,
                    pac::hart_id(),
                    alarm.hart.get(),
                    ts
                );
                let interval = if ts < now { 0 } else { ts - now };
                self._set_alarm(interval, alarm.hart.get());
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
    mpfs_hal::print_unguarded!("Hart {} timer! at {}", pac::hart_id(), DRIVER.now());

    let pending = DRIVER.trigger_alarm();

    #[cfg(feature = "debug_logs")]
    mpfs_hal::print_unguarded!("returning from timer");

    return if pending {
        pac::EXT_IRQ_KEEP_ENABLED
    } else {
        pac::EXT_IRQ_DISABLE
    } as u8;
}
