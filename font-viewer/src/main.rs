//! Font Viewer - Voyager Display Simulator
//!
//! Full display simulation: top half shows info, bottom half scrolls greetings BMP

use clap::Parser;
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::FONT_9X15, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
    sdl2::Keycode,
};
use embedded_text::{TextBox, alignment::HorizontalAlignment, style::TextBoxStyleBuilder};
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tinybmp::Bmp;
use voyager_display::Distance;

const DISPLAY_WIDTH: u32 = 128;
const DISPLAY_HEIGHT: u32 = 64;
const MESSAGE_Y: i32 = 32; // Bottom half for scrolling message

#[derive(Parser, Debug)]
#[command(name = "font-viewer")]
#[command(about = "Voyager display simulator - full display preview")]
struct Args {
    /// Path to greetings BMP file
    #[arg(default_value = "firmware/assets/greetings.bmp")]
    bmp: PathBuf,

    /// Display scale factor
    #[arg(long, default_value = "8")]
    scale: u32,
}

fn main() {
    let args = Args::parse();

    println!("Loading BMP from: {}", args.bmp.display());

    // Load BMP
    let bmp_data = std::fs::read(&args.bmp).unwrap_or_else(|e| {
        eprintln!("Error reading BMP: {}", e);
        std::process::exit(1);
    });

    let bmp = Bmp::<BinaryColor>::from_slice(&bmp_data).unwrap_or_else(|e| {
        eprintln!("Error parsing BMP: {:?}", e);
        std::process::exit(1);
    });

    let bmp_width = bmp.bounding_box().size.width as i32;
    let bmp_height = bmp.bounding_box().size.height as i32;
    println!("BMP size: {}x{} pixels", bmp_width, bmp_height);

    run_viewer(&bmp, bmp_width, args.scale);
}

fn run_viewer(bmp: &Bmp<BinaryColor>, bmp_width: i32, scale: u32) {
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .scale(scale)
        .build();

    let mut window = Window::new("Voyager Display Simulator - ESC to exit", &output_settings);

    println!("\nControls:");
    println!("  ESC/Q     - Exit");
    println!("  SPACE     - Pause/Resume scrolling");
    println!("  UP/DOWN   - Scroll speed");
    println!("  HOME      - Reset scroll");

    let mut scroll_paused = false;
    let mut scroll_offset = 0i32;
    let mut scroll_speed = 2;

    let character_style = MonoTextStyle::new(&FONT_9X15, BinaryColor::On);
    let textbox_style = TextBoxStyleBuilder::new()
        .alignment(HorizontalAlignment::Left)
        .build();

    'running: loop {
        let mut display: SimulatorDisplay<BinaryColor> =
            SimulatorDisplay::new(Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT));

        // Clear display
        Rectangle::new(Point::zero(), display.size())
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::Off))
            .draw(&mut display)
            .unwrap();

        // Get current time and calculate real Voyager 1 distance
        let unix_secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let distance = Distance::from_unix_timestamp(unix_secs);
        let delay = distance.signal_delay();

        // === Top half: Info display (2 lines, 15px each) ===
        let distance_str = distance.to_string();
        let bounds = Rectangle::new(Point::new(1, 1), Size::new(126, 15));
        TextBox::with_textbox_style(&distance_str, bounds, character_style, textbox_style)
            .draw(&mut display)
            .unwrap();

        let delay_str = delay.to_string();
        let bounds = Rectangle::new(Point::new(1, 16), Size::new(126, 15));
        TextBox::with_textbox_style(&delay_str, bounds, character_style, textbox_style)
            .draw(&mut display)
            .unwrap();

        // === Bottom half: Scrolling BMP ===
        let bmp_pos = Point::new(-scroll_offset, MESSAGE_Y);
        Image::new(bmp, bmp_pos).draw(&mut display).unwrap();

        // Seamless looping
        if scroll_offset > bmp_width - DISPLAY_WIDTH as i32 {
            let wrap_pos = Point::new(bmp_width - scroll_offset, MESSAGE_Y);
            Image::new(bmp, wrap_pos).draw(&mut display).unwrap();
        }

        // Scroll animation
        if !scroll_paused {
            scroll_offset += scroll_speed;
            if scroll_offset >= bmp_width {
                scroll_offset = 0;
            }
        }

        window.update(&display);

        // Handle events
        for event in window.events() {
            match event {
                SimulatorEvent::Quit => break 'running,
                SimulatorEvent::KeyDown { keycode, .. } => match keycode {
                    Keycode::Escape | Keycode::Q => break 'running,
                    Keycode::Space => scroll_paused = !scroll_paused,
                    Keycode::Up => {
                        scroll_speed = (scroll_speed + 1).min(20);
                        println!("Speed: {} px/frame", scroll_speed);
                    }
                    Keycode::Down => {
                        scroll_speed = (scroll_speed - 1).max(1);
                        println!("Speed: {} px/frame", scroll_speed);
                    }
                    Keycode::Home => scroll_offset = 0,
                    _ => {}
                },
                _ => {}
            }
        }

        thread::sleep(Duration::from_millis(30));
    }

    println!("Simulator closed.");
}
