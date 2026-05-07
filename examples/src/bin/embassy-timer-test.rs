#![no_std]
#![no_main]

#[macro_use]
extern crate mpfs_hal;

use embassy_time::{Duration, Instant, Timer};
use rand::{Rng, SeedableRng, rngs::SmallRng};

// Not only does this test to see how accurate our timers are, but if there are bugs in the timer driver or executor, this is intended to expose them, by triggering many alarms that are set to go off close to the period where we go into WFI. If bugs exist, we will probably never wake up.

const NUM_SAMPLES: usize = 10000;

async fn timer_test() {
    let mut rng = SmallRng::seed_from_u64(unsafe { mpfs_hal::pac::readmcycle() } as u64);

    // Cumulative delays
    let mut delays = [0; NUM_SAMPLES];
    let mut actual_delays = [0; NUM_SAMPLES];
    for i in 0..NUM_SAMPLES {
        delays[i] = rng.random_range(0..20);
        if i > 0 {
            delays[i] += delays[i - 1];
        }
    }

    let loop_start = Instant::now();
    Timer::at(loop_start + Duration::from_micros(delays[0])).await;
    for i in 1..NUM_SAMPLES {
        Timer::at(loop_start + Duration::from_micros(delays[i])).await;
        let after_delay = Instant::now();
        actual_delays[i] = after_delay.duration_since(loop_start).as_micros();
    }

    let mut delay_diffs = [0; NUM_SAMPLES];
    for i in 0..NUM_SAMPLES {
        let delay_diff = actual_delays[i] as i64 - delays[i] as i64;
        delay_diffs[i] = delay_diff;
    }
    let average_delay_diff = delay_diffs.iter().sum::<i64>() / NUM_SAMPLES as i64;
    log::info!(
        "# Timer test for hart {} completed",
        mpfs_hal::pac::hart_id()
    );
    log::info!("  Average delay diff: {} us", average_delay_diff);
    log::info!("  Max delay diff: {} us", delay_diffs.iter().max().unwrap());
    log::info!("  Min delay diff: {} us", delay_diffs.iter().min().unwrap());
}

#[mpfs_hal_embassy::embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    println!("Hello World from Rust from hart 1!");
    timer_test().await;
}

#[mpfs_hal_embassy::embassy_hart2_main]
async fn hart2_main(_spawner: embassy_executor::Spawner) {
    println!("Hello World from Rust from hart 2!");
    timer_test().await;
}

#[mpfs_hal_embassy::embassy_hart3_main]
async fn hart3_main(_spawner: embassy_executor::Spawner) {
    mpfs_hal::log_task().await;
}

#[mpfs_hal::init_once]
fn config() {
    mpfs_hal::init_logger(log::LevelFilter::Info);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    mpfs_hal::print_panic(info);
    log::error!("Panic in timer test: {:#?}", info);
    mpfs_hal::low_power_loop_forever()
}
