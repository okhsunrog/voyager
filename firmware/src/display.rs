//! OLED display rendering module.
//!
//! Drives a 128×64 SSD1306 OLED over I2C.  The display shows two things:
//!
//! 1. **Info panel** — distance and signal delay data computed from the
//!    current Unix timestamp (via the `voyager-display` crate).
//! 2. **Scrolling banner** — a pre-rendered BMP greeting image that
//!    scrolls horizontally at a fixed speed.
//!
//! The display is optional: if the SSD1306 doesn't respond during init
//! this future parks itself forever so the rest of the firmware keeps
//! running.

use crate::CURRENT_TIME;
use embassy_nrf::twim::Twim;
use embassy_time::Timer;
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use log::info;
use portable_atomic::Ordering;
use ssd1306::{Ssd1306Async, mode::BufferedGraphicsModeAsync, prelude::*};
use voyager_display::{Bmp, Distance, advance_scroll, render_info, render_scrolling_bmp};

/// The display type used throughout the firmware (I2C-backed SSD1306 128×64
/// in buffered async graphics mode).
pub type Display<'a> = Ssd1306Async<
    I2CInterface<Twim<'a>>,
    DisplaySize128x64,
    BufferedGraphicsModeAsync<DisplaySize128x64>,
>;

/// Greetings bitmap (pre-rendered by the greeting-renderer tool and
/// embedded into the binary at compile time).
static GREETINGS_BMP: &[u8] = include_bytes!("../assets/greetings.bmp");

/// How many pixels the banner scrolls per frame.
const SCROLL_SPEED: i32 = 3;

/// Initialize the display and run the rendering loop forever.
///
/// If the SSD1306 fails to initialize (e.g. not connected), this function
/// logs a warning and suspends indefinitely so it doesn't waste CPU.
pub async fn run(mut display: Display<'_>) {
    // Try to initialize the display hardware.
    let display_ok = match display.init().await {
        Ok(()) => {
            info!("[display] initialized");
            true
        }
        Err(_) => {
            info!("[display] init failed, continuing without display");
            false
        }
    };

    if !display_ok {
        // Park this future forever — the display is absent.
        core::future::pending::<()>().await;
        return;
    }

    // Parse the BMP image once.
    let bmp = Bmp::<BinaryColor>::from_slice(GREETINGS_BMP).unwrap();
    let bmp_width = bmp.bounding_box().size.width as i32;
    let mut scroll_offset = 0i32;

    // Render loop: ~20 fps (50 ms per frame).
    loop {
        let current_time = CURRENT_TIME.load(Ordering::Relaxed);

        display.clear_buffer();

        // Top section: distance + signal delay info.
        let distance = Distance::from_unix_timestamp(current_time);
        let delay = distance.signal_delay();
        let _ = render_info(&mut display, &distance, &delay);

        // Bottom section: horizontally scrolling greeting banner.
        let _ = render_scrolling_bmp(&mut display, &bmp, scroll_offset);
        scroll_offset = advance_scroll(scroll_offset, SCROLL_SPEED, bmp_width);

        let _ = display.flush().await;

        Timer::after_millis(50).await;
    }
}
