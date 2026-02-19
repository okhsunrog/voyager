//! LED blink task.
//!
//! Drives the onboard LED and three external beacon LEDs in an
//! aircraft-style double-flash pattern:
//!
//!   1. Onboard LED turns ON for 200 ms total.
//!   2. During that window the three beacon LEDs do two 50 ms flashes
//!      separated by a 50 ms gap.
//!   3. Everything turns OFF for 1800 ms before the cycle repeats.

use crate::resources::LedResources;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;

/// Embassy task that blinks all LEDs forever.
///
/// `r` contains the GPIO pins for the onboard LED and the three beacon LEDs.
/// The pins are consumed here and wrapped in [`Output`] drivers, so no other
/// code needs to know which physical pins are used.
#[embassy_executor::task]
pub async fn led_task(r: LedResources) -> ! {
    // Create output drivers — all start LOW (off).
    let mut led = Output::new(r.onboard, Level::Low, OutputDrive::Standard);
    let mut led1 = Output::new(r.led1, Level::Low, OutputDrive::Standard);
    let mut led2 = Output::new(r.led2, Level::Low, OutputDrive::Standard);
    let mut led3 = Output::new(r.led3, Level::Low, OutputDrive::Standard);

    loop {
        // --- 200 ms "on" window ---

        led.set_high(); // onboard LED on

        // Beacon flash #1 (50 ms)
        led1.set_high();
        led2.set_high();
        led3.set_high();
        Timer::after_millis(50).await;
        led1.set_low();
        led2.set_low();
        led3.set_low();

        // Gap (50 ms)
        Timer::after_millis(50).await;

        // Beacon flash #2 (50 ms)
        led1.set_high();
        led2.set_high();
        led3.set_high();
        Timer::after_millis(50).await;
        led1.set_low();
        led2.set_low();
        led3.set_low();

        // Remaining 50 ms to complete the 200 ms on-window
        Timer::after_millis(50).await;
        led.set_low(); // onboard LED off

        // --- 1800 ms pause ---
        Timer::after_millis(1800).await;
    }
}
