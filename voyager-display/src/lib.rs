//! Voyager Display Library
//!
//! Shared display rendering logic for firmware and simulator.
//! Fixed-point distance representation for precise display without FPU.

#![cfg_attr(not(feature = "std"), no_std)]

use core::fmt::{self, Write};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    image::Image,
    mono_font::{ascii::FONT_9X15, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
pub use tinybmp::Bmp;

// ============================================================================
// Voyager 1 Ephemeris Reference (from NASA JPL Horizons)
// ============================================================================
// Epoch: January 1, 2025 00:00:00 TDB (≈ UTC for our purposes)
// Range from Sun: 24,787,820,465 km = 165.6962 AU
// Range-rate: 16.88225 km/s = 1.1285 × 10⁻⁷ AU/s
//
// Query: https://ssd.jpl.nasa.gov/api/horizons.api
//        COMMAND='-31' CENTER='@sun' START_TIME='2025-01-01'
// ============================================================================

/// Reference epoch: January 1, 2025 00:00:00 UTC
/// Unix timestamp: 1735689600 (u32 valid until year 2106)
pub const EPOCH_UNIX_SECS: u32 = 1735689600;

/// Distance at epoch in units of 10^-7 AU
/// 165.6962 AU = 1,656,962,000 units
pub const EPOCH_DISTANCE_UNITS: u64 = 1_656_962_000;

/// Velocity in units of 10^-7 AU per second
/// 16.88225 km/s = 1.1285 × 10⁻⁷ AU/s
/// Using 11285 units per 10000 seconds for precision
pub const VELOCITY_UNITS_PER_10000_SEC: u64 = 11285;

/// Display dimensions (SSD1306 128x64)
pub const DISPLAY_WIDTH: u32 = 128;
pub const DISPLAY_HEIGHT: u32 = 64;

/// Fixed-point distance in units of 10^-7 AU
///
/// Examples:
/// - 170.0000000 AU = 1_700_000_000 units
/// - 0.0000001 AU = 1 unit
///
/// u64 supports up to ~1.8 × 10^12 units = ~180,000 AU (more than enough)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Distance(pub u64);

/// Fixed-point signal delay in units of 10^-5 seconds (10 microseconds)
///
/// Examples:
/// - 23:32:35.00000 = 8,475,500,000 units (84755 seconds)
/// - 0.00001 sec = 1 unit
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct SignalDelay(pub u64);

/// Scale factor for Distance (10^7)
pub const DISTANCE_SCALE: u64 = 10_000_000;

/// Scale factor for SignalDelay (10^5)
pub const DELAY_SCALE: u64 = 100_000;

/// Seconds per AU (light travel time)
/// 1 AU = 499.004784 seconds
/// In our units: 499004784 units of 10^-7 seconds... but we use 10^-5 for delay
/// So: 1 AU = 49900478 units of 10^-5 seconds
pub const DELAY_PER_AU: u64 = 49_900_478;

impl Distance {
    /// Create from AU with 7 decimal places
    /// e.g., from_au_parts(170, 0) = 170.0000000 AU
    pub const fn from_au_parts(integer: u64, fractional_7digits: u64) -> Self {
        Self(integer * DISTANCE_SCALE + fractional_7digits)
    }

    /// Create from raw units (10^-7 AU)
    pub const fn from_raw(units: u64) -> Self {
        Self(units)
    }

    /// Calculate current Voyager 1 distance from Unix timestamp
    ///
    /// Uses linear extrapolation from epoch reference point.
    /// Accurate enough for display purposes (real trajectory has minor variations).
    pub fn from_unix_timestamp(unix_secs: u32) -> Self {
        if unix_secs >= EPOCH_UNIX_SECS {
            let elapsed_secs = (unix_secs - EPOCH_UNIX_SECS) as u64;
            // distance = epoch_distance + velocity * time
            // velocity = 11285 units per 10000 seconds
            let delta_units = elapsed_secs * VELOCITY_UNITS_PER_10000_SEC / 10000;
            Self(EPOCH_DISTANCE_UNITS + delta_units)
        } else {
            // Before epoch - extrapolate backwards
            let elapsed_secs = (EPOCH_UNIX_SECS - unix_secs) as u64;
            let delta_units = elapsed_secs * VELOCITY_UNITS_PER_10000_SEC / 10000;
            Self(EPOCH_DISTANCE_UNITS.saturating_sub(delta_units))
        }
    }

    /// Get raw units
    pub const fn raw(&self) -> u64 {
        self.0
    }

    /// Get integer part of AU
    pub const fn integer_part(&self) -> u64 {
        self.0 / DISTANCE_SCALE
    }

    /// Get fractional part (7 digits, with leading zeros)
    pub const fn fractional_part(&self) -> u64 {
        self.0 % DISTANCE_SCALE
    }

    /// Add distance (e.g., for velocity integration)
    pub const fn add(&self, other: Distance) -> Self {
        Self(self.0 + other.0)
    }

    /// Calculate signal delay for this distance
    /// Returns delay in 10^-5 second units
    pub fn signal_delay(&self) -> SignalDelay {
        // delay = distance * 499.004784 sec/AU
        // In our units: delay_units = distance_units * 49900478 / 10000000
        // Simplified: delay_units = distance_units * 499 / 100 (approximate)
        // For precision: use full calculation with care for overflow

        // distance is in 10^-7 AU, we want delay in 10^-5 seconds
        // 1 AU = 499.004784 seconds = 49900478.4 units of 10^-5 sec
        // delay = distance_raw * 49900478 / 10000000

        let delay = self.0 * DELAY_PER_AU / DISTANCE_SCALE;
        SignalDelay(delay)
    }
}

impl SignalDelay {
    /// Get total seconds (integer part)
    pub const fn total_seconds(&self) -> u64 {
        self.0 / DELAY_SCALE
    }

    /// Get fractional part (5 digits after decimal in seconds)
    pub const fn fractional_part(&self) -> u64 {
        self.0 % DELAY_SCALE
    }

    /// Get hours component
    pub const fn hours(&self) -> u64 {
        self.total_seconds() / 3600
    }

    /// Get minutes component (0-59)
    pub const fn minutes(&self) -> u64 {
        (self.total_seconds() % 3600) / 60
    }

    /// Get seconds component (0-59)
    pub const fn seconds(&self) -> u64 {
        self.total_seconds() % 60
    }
}

/// Format Distance as "XXX.XXXXXXX AU"
impl fmt::Display for Distance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{:07} AU", self.integer_part(), self.fractional_part())
    }
}

/// Format SignalDelay as "HH:MM:SS.XXXXX"
impl fmt::Display for SignalDelay {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:02}:{:02}:{:02}.{:05}",
            self.hours(),
            self.minutes(),
            self.seconds(),
            self.fractional_part()
        )
    }
}

/// Helper to format Distance into a fixed-size buffer (no_std compatible)
pub fn format_distance<const N: usize>(dist: &Distance, buf: &mut heapless::String<N>) {
    let _ = write!(buf, "{}.{:07} AU", dist.integer_part(), dist.fractional_part());
}

/// Helper to format SignalDelay into a fixed-size buffer (no_std compatible)
pub fn format_delay<const N: usize>(delay: &SignalDelay, buf: &mut heapless::String<N>) {
    let _ = write!(
        buf,
        "{:02}:{:02}:{:02}.{:05}",
        delay.hours(),
        delay.minutes(),
        delay.seconds(),
        delay.fractional_part()
    );
}

// ============================================================================
// Display Rendering
// ============================================================================

/// Y position where the bottom half (scrolling message) starts
pub const MESSAGE_Y: i32 = 32;

/// Render the info section (top half) showing distance and signal delay
pub fn render_info<D>(display: &mut D, distance: &Distance, delay: &SignalDelay) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    let text_style = MonoTextStyle::new(&FONT_9X15, BinaryColor::On);

    // Format distance
    let mut dist_buf: heapless::String<32> = heapless::String::new();
    format_distance(distance, &mut dist_buf);

    // Format delay
    let mut delay_buf: heapless::String<32> = heapless::String::new();
    format_delay(delay, &mut delay_buf);

    // Draw distance (line 1, y=1)
    embedded_graphics::text::Text::new(&dist_buf, Point::new(1, 14), text_style)
        .draw(display)?;

    // Draw delay (line 2, y=16)
    embedded_graphics::text::Text::new(&delay_buf, Point::new(1, 30), text_style)
        .draw(display)?;

    Ok(())
}

/// Render the scrolling BMP in the bottom half of the display
pub fn render_scrolling_bmp<D>(
    display: &mut D,
    bmp: &Bmp<'_, BinaryColor>,
    scroll_offset: i32,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    let bmp_width = bmp.bounding_box().size.width as i32;

    // Draw BMP at current scroll position
    let bmp_pos = Point::new(-scroll_offset, MESSAGE_Y);
    Image::new(bmp, bmp_pos).draw(display)?;

    // Seamless looping: draw second copy when needed
    if scroll_offset > bmp_width - DISPLAY_WIDTH as i32 {
        let wrap_pos = Point::new(bmp_width - scroll_offset, MESSAGE_Y);
        Image::new(bmp, wrap_pos).draw(display)?;
    }

    Ok(())
}

/// Clear the entire display
pub fn clear_display<D>(display: &mut D) -> Result<(), D::Error>
where
    D: DrawTarget<Color = BinaryColor>,
{
    Rectangle::new(Point::zero(), Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(display)
}

/// Calculate the next scroll offset for animation
pub fn advance_scroll(scroll_offset: i32, speed: i32, bmp_width: i32) -> i32 {
    let new_offset = scroll_offset + speed;
    if new_offset >= bmp_width {
        0
    } else {
        new_offset
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;
    use alloc::string::ToString;

    #[test]
    fn test_distance_display() {
        let d = Distance::from_au_parts(170, 1234567);
        assert_eq!(d.to_string(), "170.1234567 AU");
    }

    #[test]
    fn test_distance_zero_padding() {
        let d = Distance::from_au_parts(170, 123);
        assert_eq!(d.to_string(), "170.0000123 AU");
    }

    #[test]
    fn test_signal_delay() {
        let d = Distance::from_au_parts(1, 0); // 1 AU
        let delay = d.signal_delay();
        // Should be ~499 seconds = 00:08:19.xxxxx
        assert_eq!(delay.hours(), 0);
        assert_eq!(delay.minutes(), 8);
        assert_eq!(delay.seconds(), 19);
    }

    #[test]
    fn test_signal_delay_display() {
        let d = Distance::from_au_parts(170, 0);
        let delay = d.signal_delay();
        let s = delay.to_string();
        // 170 AU * 499 sec/AU = 84830 sec = 23:33:50
        assert!(s.starts_with("23:33:"));
    }

    #[test]
    fn test_scroll_advance() {
        assert_eq!(advance_scroll(0, 2, 100), 2);
        assert_eq!(advance_scroll(98, 2, 100), 0); // wraps
        assert_eq!(advance_scroll(50, 2, 100), 52);
    }
}
