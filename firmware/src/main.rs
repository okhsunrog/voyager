#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_time::{Instant, Timer};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use fmt::info;
use portable_atomic::{AtomicU32, Ordering};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};
use static_cell::StaticCell;
use voyager_display::{Bmp, Distance, advance_scroll, render_info, render_scrolling_bmp};

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

/// LED blink task - 200ms on, 1800ms off (2 second cycle)
#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(200).await;
        led.set_low();
        Timer::after_millis(1800).await;
    }
}

/// Parse a decimal string to u32 at compile time
const fn parse_u32(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut result: u32 = 0;
    let mut i = 0;
    while i < bytes.len() {
        let digit = bytes[i] - b'0';
        result = result * 10 + digit as u32;
        i += 1;
    }
    result
}

/// Compile-time Unix timestamp from build.rs
const BUILD_TIMESTAMP: u32 = parse_u32(env!("BUILD_TIMESTAMP"));

/// Current time, initialized from compile time and incremented every second
static CURRENT_TIME: AtomicU32 = AtomicU32::new(BUILD_TIMESTAMP);

/// Greetings bitmap (pre-rendered with greeting-renderer)
static GREETINGS_BMP: &[u8] = include_bytes!("../assets/greetings.bmp");

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // P0.13 controls VCC on nice!nano - set HIGH to enable 3.3V power
    let _vcc_enable = Output::new(p.P0_13, Level::High, OutputDrive::Standard);

    // Setup LED blink task
    let led = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);
    spawner.spawn(led_task(led)).unwrap();

    // Wait for display to power up
    Timer::after_millis(100).await;

    // Setup async I2C at 400kHz
    static TWIM_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let mut twim_config = embassy_nrf::twim::Config::default();
    twim_config.frequency = embassy_nrf::twim::Frequency::K400;
    let i2c = Twim::new(p.TWISPI0, Irqs, p.P0_17, p.P0_20, twim_config, TWIM_BUF.init([0; 2048]));

    // Initialize SSD1306 display
    let mut display = Ssd1306Async::new(I2CDisplayInterface::new(i2c), DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().await.unwrap();

    info!("Voyager display initialized (build timestamp: {})", BUILD_TIMESTAMP);

    // Load BMP
    let bmp = Bmp::<BinaryColor>::from_slice(GREETINGS_BMP).unwrap();
    let bmp_width = bmp.bounding_box().size.width as i32;

    let mut scroll_offset = 0i32;
    const SCROLL_SPEED: i32 = 3;

    let start_time = Instant::now();

    loop {
        // Calculate current timestamp (build time + elapsed seconds)
        let elapsed_secs = start_time.elapsed().as_secs() as u32;
        let current_time = BUILD_TIMESTAMP + elapsed_secs;
        CURRENT_TIME.store(current_time, Ordering::Relaxed);

        display.clear_buffer();

        // Calculate Voyager 1 distance from current timestamp
        let distance = Distance::from_unix_timestamp(current_time);
        let delay = distance.signal_delay();
        let _ = render_info(&mut display, &distance, &delay);

        // Render scrolling greetings
        let _ = render_scrolling_bmp(&mut display, &bmp, scroll_offset);
        scroll_offset = advance_scroll(scroll_offset, SCROLL_SPEED, bmp_width);

        let _ = display.flush().await;

        Timer::after_millis(50).await;
    }
}
