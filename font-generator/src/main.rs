use anyhow::{Context, Result, bail};
use clap::Parser;
use fontdue::{Font, FontSettings};
use std::collections::BTreeSet;
use std::fs;
use std::io::Write;
use std::path::PathBuf;

/// Magic bytes for font format version 2 (with configurable size)
const MAGIC: &[u8; 4] = b"FNT2";
const MISSING_GLYPH: u32 = 0xFFFFFFFF;

/// Header size: magic(4) + width(1) + height(1) + reserved(2) + glyph_count(4) + non_ascii_count(4) = 16
const HEADER_SIZE: usize = 16;
const ASCII_TABLE_SIZE: usize = 128 * 4;

#[derive(Parser, Debug)]
#[command(name = "font_generator")]
#[command(about = "Generate bitmap font for embedded systems")]
struct Args {
    /// Comma-separated list of font file paths (tried in order for each glyph)
    #[arg(short, long, value_delimiter = ',')]
    fonts: Vec<PathBuf>,

    /// Text file with additional characters to include
    #[arg(short, long)]
    text: Option<PathBuf>,

    /// Output binary file path
    #[arg(short, long, default_value = "font.bin")]
    output: PathBuf,

    /// Font size (width and height in pixels, e.g., "16" for 16x16 or "8x12" for 8 wide, 12 tall)
    #[arg(short, long, default_value = "16")]
    size: String,

    /// Threshold for converting grayscale to 1-bit (0-255)
    #[arg(long, default_value = "128")]
    threshold: u8,

    /// Check if all glyphs can be rendered (don't generate output)
    #[arg(long)]
    check: bool,
}

fn parse_size(size_str: &str) -> Result<(u8, u8)> {
    if let Some((w, h)) = size_str.split_once('x') {
        let width: u8 = w.parse().context("Invalid width")?;
        let height: u8 = h.parse().context("Invalid height")?;
        Ok((width, height))
    } else {
        let size: u8 = size_str.parse().context("Invalid size")?;
        Ok((size, size))
    }
}

fn bytes_per_row(width: u8) -> usize {
    (width as usize + 7) / 8
}

fn bytes_per_glyph(width: u8, height: u8) -> usize {
    bytes_per_row(width) * height as usize
}

fn collect_codepoints(text_file: Option<&PathBuf>) -> Result<BTreeSet<char>> {
    let mut codepoints = BTreeSet::new();

    // ASCII (0x00-0x7F)
    for cp in 0x00u32..=0x7F {
        if let Some(c) = char::from_u32(cp) {
            codepoints.insert(c);
        }
    }

    // Cyrillic (0x0400-0x04FF)
    for cp in 0x0400u32..=0x04FF {
        if let Some(c) = char::from_u32(cp) {
            codepoints.insert(c);
        }
    }

    // Characters from text file
    if let Some(path) = text_file {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read text file: {}", path.display()))?;
        for c in content.chars() {
            codepoints.insert(c);
        }
    }

    Ok(codepoints)
}

fn load_fonts(paths: &[PathBuf]) -> Result<Vec<Font>> {
    let mut fonts = Vec::new();
    for path in paths {
        let data = fs::read(path)
            .with_context(|| format!("Failed to read font file: {}", path.display()))?;
        let font = Font::from_bytes(data, FontSettings::default())
            .map_err(|e| anyhow::anyhow!("Failed to parse font {}: {}", path.display(), e))?;
        fonts.push(font);
    }
    Ok(fonts)
}

fn is_control_char(c: char) -> bool {
    let cp = c as u32;
    cp < 0x20 || cp == 0x7F || (0x80..=0x9F).contains(&cp)
}

fn rasterize_glyph(
    fonts: &[Font],
    c: char,
    width: u8,
    height: u8,
    threshold: u8,
) -> Option<Vec<u8>> {
    let w = width as usize;
    let h = height as usize;
    let bpr = bytes_per_row(width);
    let bpg = bytes_per_glyph(width, height);

    // Control characters get empty glyphs
    if is_control_char(c) {
        return Some(vec![0u8; bpg]);
    }

    for font in fonts {
        if !font.has_glyph(c) {
            continue;
        }

        // Rasterize at the larger dimension to get good coverage
        let raster_size = w.max(h) as f32;
        let (metrics, bitmap) = font.rasterize(c, raster_size);

        let mut packed = vec![0u8; bpg];

        // Calculate offset to center the glyph
        let x_offset = if metrics.width < w {
            (w - metrics.width) / 2
        } else {
            0
        };
        let y_offset = if metrics.height < h {
            (h - metrics.height) / 2
        } else {
            0
        };

        for y in 0..metrics.height.min(h) {
            for x in 0..metrics.width.min(w) {
                let src_idx = y * metrics.width + x;
                if src_idx < bitmap.len() && bitmap[src_idx] >= threshold {
                    let dest_x = x + x_offset;
                    let dest_y = y + y_offset;
                    if dest_x < w && dest_y < h {
                        let row_idx = dest_y * bpr;
                        let byte_idx = dest_x / 8;
                        let bit_idx = 7 - (dest_x % 8);
                        packed[row_idx + byte_idx] |= 1 << bit_idx;
                    }
                }
            }
        }

        return Some(packed);
    }
    None
}

fn check_glyphs(
    codepoints: &BTreeSet<char>,
    fonts: &[Font],
    width: u8,
    height: u8,
    threshold: u8,
) -> Result<()> {
    let mut missing: Vec<char> = Vec::new();

    println!("\nChecking glyphs...");

    // Check all glyphs
    for &c in codepoints {
        if rasterize_glyph(fonts, c, width, height, threshold).is_none() {
            missing.push(c);
        }
    }

    // Report results
    if !missing.is_empty() {
        eprintln!("\n✗ Missing glyphs ({} codepoints):", missing.len());
        for c in &missing {
            eprintln!("  U+{:04X} '{}' - {}", *c as u32, c.escape_unicode(), c);
        }
        bail!(
            "Cannot render font: {} codepoints are missing from all provided fonts",
            missing.len()
        );
    }

    Ok(())
}

fn write_binary_output(
    output_path: &PathBuf,
    codepoints: &BTreeSet<char>,
    fonts: &[Font],
    width: u8,
    height: u8,
    threshold: u8,
) -> Result<()> {
    let bpg = bytes_per_glyph(width, height);

    let mut glyphs: Vec<(char, Vec<u8>)> = Vec::new();
    let mut missing: Vec<char> = Vec::new();

    // Rasterize all glyphs
    for &c in codepoints {
        if let Some(glyph) = rasterize_glyph(fonts, c, width, height, threshold) {
            glyphs.push((c, glyph));
        } else {
            missing.push(c);
        }
    }

    // Report missing glyphs
    if !missing.is_empty() {
        eprintln!("Missing glyphs ({} codepoints):", missing.len());
        for c in &missing {
            eprintln!("  U+{:04X} '{}'", *c as u32, c.escape_unicode());
        }
        bail!(
            "Cannot generate font: {} codepoints are missing from all provided fonts",
            missing.len()
        );
    }

    // Build output
    let mut output = Vec::new();

    // Header (16 bytes)
    output.extend_from_slice(MAGIC);
    output.push(width);
    output.push(height);
    output.extend_from_slice(&[0u8; 2]); // reserved
    output.extend_from_slice(&(glyphs.len() as u32).to_le_bytes());

    let non_ascii_count = glyphs.iter().filter(|(c, _)| *c as u32 > 0x7F).count() as u32;
    output.extend_from_slice(&non_ascii_count.to_le_bytes());

    // Calculate offsets
    let non_ascii_table_size = non_ascii_count as usize * 8;
    let glyph_data_start = HEADER_SIZE + ASCII_TABLE_SIZE + non_ascii_table_size;

    // Build glyph index map
    let mut glyph_offsets: std::collections::HashMap<char, u32> = std::collections::HashMap::new();
    for (idx, (c, _)) in glyphs.iter().enumerate() {
        glyph_offsets.insert(*c, (glyph_data_start + idx * bpg) as u32);
    }

    // ASCII table (128 entries)
    for cp in 0x00u32..=0x7F {
        let c = char::from_u32(cp).unwrap();
        let offset = glyph_offsets.get(&c).copied().unwrap_or(MISSING_GLYPH);
        output.extend_from_slice(&offset.to_le_bytes());
    }

    // Non-ASCII lookup table (sorted by codepoint)
    let mut non_ascii: Vec<_> = glyphs
        .iter()
        .filter(|(c, _)| *c as u32 > 0x7F)
        .map(|(c, _)| (*c as u32, *glyph_offsets.get(c).unwrap()))
        .collect();
    non_ascii.sort_by_key(|(cp, _)| *cp);

    for (cp, offset) in &non_ascii {
        output.extend_from_slice(&cp.to_le_bytes());
        output.extend_from_slice(&offset.to_le_bytes());
    }

    // Glyph data
    for (_, glyph) in &glyphs {
        output.extend_from_slice(glyph);
    }

    // Write to file
    let mut file = fs::File::create(output_path)
        .with_context(|| format!("Failed to create output file: {}", output_path.display()))?;
    file.write_all(&output)
        .with_context(|| format!("Failed to write output file: {}", output_path.display()))?;

    println!(
        "Generated {}x{} font with {} glyphs",
        width,
        height,
        glyphs.len()
    );
    println!(
        "  ASCII glyphs: {}",
        glyphs.iter().filter(|(c, _)| (*c as u32) <= 0x7F).count()
    );
    println!("  Non-ASCII glyphs: {}", non_ascii_count);
    println!("  Bytes per glyph: {}", bpg);
    println!("  Output size: {} bytes", output.len());
    println!("  Output file: {}", output_path.display());

    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.fonts.is_empty() {
        bail!("At least one font file must be provided with --fonts");
    }

    let (width, height) = parse_size(&args.size)?;
    if width == 0 || height == 0 {
        bail!("Font size must be at least 1x1");
    }
    if width > 64 || height > 64 {
        bail!("Font size must be at most 64x64");
    }

    // Collect all required codepoints
    let codepoints = collect_codepoints(args.text.as_ref())?;
    println!("Collected {} unique codepoints", codepoints.len());

    // Load fonts
    let fonts = load_fonts(&args.fonts)?;
    println!("Loaded {} font(s)", fonts.len());

    if args.check {
        // Check mode: just verify all glyphs can be rendered
        check_glyphs(&codepoints, &fonts, width, height, args.threshold)?;
        println!("\n✓ All glyphs can be rendered successfully!");
    } else {
        // Generate output
        write_binary_output(
            &args.output,
            &codepoints,
            &fonts,
            width,
            height,
            args.threshold,
        )?;
    }

    Ok(())
}
