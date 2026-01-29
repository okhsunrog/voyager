//! Greeting Renderer - Pre-renders greetings to bitmap for embedded display
//!
//! Uses Parley for full Unicode text layout (RTL, complex scripts, shaping).
//! Output is a 1-bit BMP that firmware and simulator can use directly.

use bmp_monochrome::Bmp;
use clap::Parser;
use parley::{
    Alignment, AlignmentOptions, FontContext, GenericFamily, GlyphRun, Layout, LayoutContext,
    LineHeight, PositionedLayoutItem, StyleProperty,
};
use skrifa::{
    GlyphId, MetadataProvider,
    instance::{LocationRef, NormalizedCoord, Size},
    outline::{DrawSettings, OutlineGlyph, OutlinePen},
    raw::FontRef as ReadFontsRef,
};
use std::path::PathBuf;
use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, PixmapMut, Transform};

const DISPLAY_HEIGHT: u32 = 32;
const SEPARATOR: &str = " • ";

/// Brush type for Parley - must implement Default
#[derive(Clone, Copy, Debug, PartialEq)]
struct Brush {
    color: Color,
}

impl Default for Brush {
    fn default() -> Self {
        Self {
            color: Color::WHITE,
        }
    }
}

#[derive(Parser, Debug)]
#[command(name = "greeting-renderer")]
#[command(about = "Pre-render greetings to bitmap for embedded display")]
struct Args {
    /// Input greetings file (format: "Language: message")
    #[arg(short, long, default_value = "font-generator/greetings.txt")]
    input: PathBuf,

    /// Output BMP file for firmware
    #[arg(short, long, default_value = "firmware/assets/greetings.bmp")]
    output: PathBuf,

    /// Font size in pixels
    #[arg(short, long, default_value = "24")]
    size: f32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Load greetings
    let greetings_text = std::fs::read_to_string(&args.input)?;
    let messages: Vec<&str> = greetings_text
        .lines()
        .filter(|line| !line.trim().is_empty())
        .map(|line| {
            if let Some(pos) = line.find(':') {
                line[pos + 1..].trim()
            } else {
                line.trim()
            }
        })
        .collect();

    println!("Loaded {} greetings", messages.len());

    // Join with separator
    let full_text = messages.join(SEPARATOR);
    println!("Total text: {} chars", full_text.chars().count());

    // Setup Parley
    let mut font_cx = FontContext::new();
    let mut layout_cx = LayoutContext::new();

    // Build layout
    let display_scale = 1.0;
    let quantize = true;
    let mut builder = layout_cx.ranged_builder(&mut font_cx, &full_text, display_scale, quantize);

    // Set default styles
    builder.push_default(StyleProperty::Brush(Brush::default()));
    builder.push_default(GenericFamily::SansSerif);
    builder.push_default(StyleProperty::FontSize(args.size));
    builder.push_default(LineHeight::Absolute(DISPLAY_HEIGHT as f32));

    // Build layout
    let mut layout: Layout<Brush> = builder.build(&full_text);
    layout.break_all_lines(None); // No max width - single line
    layout.align(None, Alignment::Start, AlignmentOptions::default());

    let width = layout.width().ceil() as u32;
    let height = DISPLAY_HEIGHT;

    println!("Layout size: {}x{} pixels", width, height);

    // Render to pixmap
    let mut pixmap = Pixmap::new(width.max(1), height).ok_or("Failed to create pixmap")?;
    pixmap.fill(Color::BLACK);

    // Render glyphs
    let mut pen = TinySkiaPen::new(pixmap.as_mut());
    for line in layout.lines() {
        for item in line.items() {
            if let PositionedLayoutItem::GlyphRun(glyph_run) = item {
                render_glyph_run(&glyph_run, &mut pen);
            }
        }
    }

    // Convert to 1-bit BMP and save
    let bmp = to_bmp(&pixmap)?;
    let file = std::fs::File::create(&args.output)?;
    bmp.write(file)?;

    let file_size = std::fs::metadata(&args.output)?.len();
    let size_kb = file_size as f32 / 1024.0;
    println!(
        "Saved {} ({:.1} KB, {} bytes)",
        args.output.display(),
        size_kb,
        file_size
    );

    Ok(())
}

fn render_glyph_run(glyph_run: &GlyphRun<'_, Brush>, pen: &mut TinySkiaPen<'_>) {
    let mut run_x = glyph_run.offset();
    let run_y = glyph_run.baseline();
    let style = glyph_run.style();
    let color = style.brush.color;

    let run = glyph_run.run();
    let font = run.font();
    let font_size = run.font_size();

    let normalized_coords: Vec<NormalizedCoord> = run
        .normalized_coords()
        .iter()
        .map(|coord| NormalizedCoord::from_bits(*coord))
        .collect();

    // Get font outlines
    let font_data = font.data.as_ref();
    let font_ref = ReadFontsRef::from_index(font_data, font.index).unwrap();
    let outlines = font_ref.outline_glyphs();

    for glyph in glyph_run.glyphs() {
        let glyph_x = run_x + glyph.x;
        let glyph_y = run_y - glyph.y;
        run_x += glyph.advance;

        let glyph_id = GlyphId::from(glyph.id);
        if let Some(glyph_outline) = outlines.get(glyph_id) {
            pen.set_origin(glyph_x, glyph_y);
            pen.set_color(color);
            pen.draw_glyph(&glyph_outline, font_size, &normalized_coords);
        }
    }
}

struct TinySkiaPen<'a> {
    pixmap: PixmapMut<'a>,
    x: f32,
    y: f32,
    paint: Paint<'static>,
    open_path: PathBuilder,
}

impl TinySkiaPen<'_> {
    fn new(pixmap: PixmapMut<'_>) -> TinySkiaPen<'_> {
        TinySkiaPen {
            pixmap,
            x: 0.0,
            y: 0.0,
            paint: Paint::default(),
            open_path: PathBuilder::new(),
        }
    }

    fn set_origin(&mut self, x: f32, y: f32) {
        self.x = x;
        self.y = y;
    }

    fn set_color(&mut self, color: Color) {
        self.paint.set_color(color);
    }

    fn draw_glyph(
        &mut self,
        glyph: &OutlineGlyph<'_>,
        size: f32,
        normalized_coords: &[NormalizedCoord],
    ) {
        let location_ref = LocationRef::new(normalized_coords);
        let settings = DrawSettings::unhinted(Size::new(size), location_ref);
        let _ = glyph.draw(settings, self);

        let builder = core::mem::replace(&mut self.open_path, PathBuilder::new());
        if let Some(path) = builder.finish() {
            self.pixmap.fill_path(
                &path,
                &self.paint,
                FillRule::Winding,
                Transform::identity(),
                None,
            );
        }
    }
}

impl OutlinePen for TinySkiaPen<'_> {
    fn move_to(&mut self, x: f32, y: f32) {
        self.open_path.move_to(self.x + x, self.y - y);
    }

    fn line_to(&mut self, x: f32, y: f32) {
        self.open_path.line_to(self.x + x, self.y - y);
    }

    fn quad_to(&mut self, cx0: f32, cy0: f32, x: f32, y: f32) {
        self.open_path
            .quad_to(self.x + cx0, self.y - cy0, self.x + x, self.y - y);
    }

    fn curve_to(&mut self, cx0: f32, cy0: f32, cx1: f32, cy1: f32, x: f32, y: f32) {
        self.open_path.cubic_to(
            self.x + cx0,
            self.y - cy0,
            self.x + cx1,
            self.y - cy1,
            self.x + x,
            self.y - y,
        );
    }

    fn close(&mut self) {
        self.open_path.close();
    }
}

/// Convert RGBA pixmap to 1-bit BMP (black background, white text)
fn to_bmp(pixmap: &Pixmap) -> Result<Bmp, Box<dyn std::error::Error>> {
    let width = pixmap.width() as usize;
    let height = pixmap.height() as usize;

    // Build rows (Vec<Vec<bool>>)
    // In bmp-monochrome: false = white (palette[0]), true = black (palette[1])
    // We want: black background, white text
    // So: background (dark pixels) → true, text (bright pixels) → false
    let mut rows = Vec::with_capacity(height);
    for y in 0..height {
        let mut row = Vec::with_capacity(width);
        for x in 0..width {
            let pixel = pixmap.pixel(x as u32, y as u32).unwrap();
            // Bright pixel (text) → false (white in BMP)
            // Dark pixel (background) → true (black in BMP)
            let is_dark = pixel.red() <= 127 && pixel.green() <= 127 && pixel.blue() <= 127;
            row.push(is_dark);
        }
        rows.push(row);
    }

    Ok(Bmp::new(rows)?)
}
