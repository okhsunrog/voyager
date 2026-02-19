//! Battery voltage measurement task.
//!
//! Reads the nRF52840's internal VDDH/5 channel through the SAADC
//! (Successive Approximation ADC) every 60 seconds and stores the result
//! in the shared [`BATTERY_MV`](crate::BATTERY_MV) atomic.
//!
//! No external components are needed — the chip divides VDDH (the raw
//! battery/USB voltage) by 5 internally and feeds it to the ADC.

use crate::BATTERY_MV;
use embassy_nrf::saadc::Saadc;
use embassy_time::{Duration, Timer};
use log::info;
use portable_atomic::Ordering;

/// Embassy task that samples battery voltage forever.
///
/// The caller constructs the [`Saadc`] driver (with the VDDH/5 channel
/// and 16× hardware oversampling) and passes it here.  This task owns the
/// driver for its entire lifetime.
///
/// ## ADC math
///
/// - Internal reference: 0.6 V, gain 1/6 → full-scale input = 3.6 V
/// - 12-bit resolution → 4096 counts at 3.6 V
/// - The VDDH/5 channel reads VDDH divided by 5, so we multiply back
///   by 5 to recover the true battery voltage:
///
///   `battery_mv = raw * 3600 / 4096 * 5`
#[embassy_executor::task]
pub async fn battery_task(mut saadc: Saadc<'static, 1>) {
    loop {
        let mut buf = [0i16; 1];
        saadc.sample(&mut buf).await;

        // Clamp negative values (noise) to zero.
        let raw = buf[0].max(0) as u32;
        let battery_mv = raw * 3600 / 4096 * 5;

        BATTERY_MV.store(battery_mv, Ordering::Relaxed);
        info!("[battery] {}mV (raw={})", battery_mv, raw);

        Timer::after(Duration::from_secs(60)).await;
    }
}
