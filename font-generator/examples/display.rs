//! Example: Display text using the bitmap font with embedded-graphics-simulator.
//!
//! Run with:
//! ```
//! cargo run --example display
//! ```

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, Window,
};
use font_generator::{BitmapFont, Text};

fn main() {
    // Load the font from the generated binary file
    let font_data = std::fs::read("font.bin").expect(
        "Failed to read font.bin. Generate it first with:\n\
         cargo run -- --fonts /path/to/font.ttf --text greetings.txt",
    );

    // Load with compile-time size verification (16x16 font)
    let font: BitmapFont<16, 16> =
        BitmapFont::from_bytes_checked(&font_data).expect("Invalid font file or size mismatch");

    println!(
        "Loaded {}x{} font with {} glyphs",
        font.width(),
        font.height(),
        font.glyph_count()
    );

    // Create a display
    let mut display: SimulatorDisplay<BinaryColor> = SimulatorDisplay::new(Size::new(640, 480));

    // Clear display to white
    Rectangle::new(Point::zero(), display.size())
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
        .draw(&mut display)
        .unwrap();

    // Draw various text samples
    let samples = [
        "Hello, World!",
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
        "abcdefghijklmnopqrstuvwxyz",
        "0123456789 !@#$%^&*()",
        // Cyrillic
        "Привет мир! Здравствуйте!",
        // Greek
        "Χαίρετε! Εἰρήνη",
        // Various scripts from greetings.txt
        "日本語: こんにちは",
        "中文: 你好",
        "한국어: 안녕하세요",
        "العربية: مرحبا",
        "עברית: שלום",
        "ไทย: สวัสดี",
    ];

    let mut y = 10;
    for sample in &samples {
        Text::new(sample, &font, Point::new(10, y))
            .draw(&mut display)
            .unwrap();
        y += font.height() as i32 + 4;
    }

    // Create window and run
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .scale(1)
        .build();

    Window::new("Bitmap Font Demo", &output_settings).show_static(&display);
}
