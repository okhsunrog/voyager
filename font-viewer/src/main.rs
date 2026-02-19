//! Font Viewer - Voyager Display Simulator
//!
//! Full display simulation using shared rendering from voyager-display.

use clap::Parser;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics_simulator::{
    BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
    sdl2::Keycode,
};
use std::path::PathBuf;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use voyager_display::{
    Bmp, DISPLAY_HEIGHT, DISPLAY_WIDTH, Distance, advance_scroll, clear_display, render_info,
    render_scrolling_bmp,
};

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
    println!(
        "BMP size: {}x{} pixels",
        bmp_width,
        bmp.bounding_box().size.height
    );

    run_viewer(&bmp, bmp_width, args.scale);
}

fn run_viewer(bmp: &Bmp<BinaryColor>, bmp_width: i32, scale: u32) {
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .scale(scale)
        .build();

    let mut window = Window::new("Voyager Display Simulator - ESC to exit", &output_settings);
    let mut display: SimulatorDisplay<BinaryColor> =
        SimulatorDisplay::new(Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT));

    println!("\nControls:");
    println!("  ESC/Q     - Exit");
    println!("  SPACE     - Pause/Resume scrolling");
    println!("  UP/DOWN   - Scroll speed");
    println!("  HOME      - Reset scroll");

    let mut scroll_paused = false;
    let mut scroll_offset = 0i32;
    let mut scroll_speed = 2;
    let mut last_unix_secs = 0u64;

    'running: loop {
        // Get current time
        let unix_secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();

        // Only redraw if time changed or scrolling
        let needs_redraw = !scroll_paused || unix_secs != last_unix_secs;
        last_unix_secs = unix_secs;

        if needs_redraw {
            // Clear display
            clear_display(&mut display).unwrap();

            // Calculate real Voyager 1 distance
            let distance = Distance::from_unix_timestamp(unix_secs as u32);
            let delay = distance.signal_delay();

            // Render info section (top half)
            render_info(&mut display, &distance, &delay).unwrap();

            // Render scrolling BMP (bottom half)
            render_scrolling_bmp(&mut display, bmp, scroll_offset).unwrap();

            // Scroll animation
            if !scroll_paused {
                scroll_offset = advance_scroll(scroll_offset, scroll_speed, bmp_width);
            }

            window.update(&display);
        }

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

        // 60ms = ~16 FPS, enough for smooth scrolling while saving CPU
        thread::sleep(Duration::from_millis(60));
    }

    println!("Simulator closed.");
}
