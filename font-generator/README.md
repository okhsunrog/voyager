# Font Generator

A tool for generating bitmap fonts for embedded systems with `no_std` support.

## Features

- Generate bitmap fonts from TrueType/OpenType fonts
- Configurable font dimensions (e.g., 8x8, 16x16, 8x12)
- Support for ASCII, Cyrillic, and custom character sets
- Compile-time size verification
- Build-time font validation
- `no_std` compatible library for embedded use
- Integration with `embedded-graphics`

## Usage

### Generating a Font

Use the `just` command to generate a font:

```bash
# Generate a 16x16 font from system fonts
just generate-font /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf

# Generate with custom size
just generate-font /path/to/font.ttf 8x12

# Generate with additional characters from a text file
just generate-font /path/to/font.ttf 16 greetings.txt
```

Or use cargo directly:

```bash
cargo run -p font_generator --release -- \
    --fonts /path/to/font1.ttf,/path/to/font2.ttf \
    --size 16 \
    --text greetings.txt \
    --output firmware/assets/font.bin
```

### Font Format

The generated font file uses a binary format:
- ASCII characters (0x00-0x7F) for fast direct lookup
- Cyrillic and other Unicode characters via binary search
- Header includes font dimensions and glyph count
- Bitmap data is 1-bit per pixel, packed MSB first

### Using in Firmware

The font generator library is `no_std` compatible for use in embedded firmware.

**In your firmware code:**

```rust
use embedded_graphics::prelude::*;

// Embed font at compile time
static FONT_DATA: &[u8] = include_bytes!("../assets/font.bin");

// Load font (runtime size verification)
let font_info = font_generator::read_font_info(FONT_DATA)
    .expect("Invalid font file");

// Or with compile-time size checking:
let font: font_generator::BitmapFont<16, 16> = 
    font_generator::BitmapFont::from_bytes_checked(FONT_DATA)
        .expect("Font size mismatch");

// Draw text using embedded-graphics
use font_generator::Text;
Text::new("Hello, World!", &font, Point::new(10, 10))
    .draw(&mut display)?;

// Or use WrappedText for line wrapping:
use font_generator::WrappedText;
WrappedText::new("Long text...", &font, Point::new(10, 10), 200)
    .draw(&mut display)?;
```

**Optional: Build-time validation (requires `std` for build scripts):**

If you want to validate the font at build time, add font_generator as a build dependency in a separate build.rs:

```toml
[build-dependencies]
font_generator = { path = "../font-generator" }
```

```rust
// build.rs
fn main() {
    println!("cargo:rerun-if-changed=assets/font.bin");
    
    // Validate font dimensions at build time
    font_generator::validate_font_file::<16, 16>("assets/font.bin")
        .expect("Font validation failed");
}
```

**Note:** Build-time validation requires your build environment to support `std`, but the runtime library (used in firmware) is fully `no_std` compatible.

### Preview Font

View the generated font in a SSD1306 OLED simulator (128x64 pixels):

```bash
just view-font
```

This uses the `font-viewer` crate to display the font exactly as it will appear on the actual hardware.

For more details, see the [font-viewer README](../font-viewer/README.md).

## Examples

See `examples/` directory for:
- `display.rs` - Render text with the simulator
- `build_rs_example.rs` - Build-time validation example

## Font Format Details

**Header (16 bytes):**
- Magic: `FNT2` (4 bytes)
- Width: u8 (1 byte)
- Height: u8 (1 byte)
- Reserved: 2 bytes
- Glyph count: u32 LE (4 bytes)
- Non-ASCII count: u32 LE (4 bytes)

**ASCII Table (512 bytes):**
- 128 entries × 4 bytes (u32 LE offsets)
- 0xFFFFFFFF = missing glyph

**Non-ASCII Table (variable):**
- Sorted by codepoint
- Each entry: codepoint (u32 LE) + offset (u32 LE)

**Glyph Data:**
- 1-bit packed bitmap
- MSB first, padded to byte boundary
- Row-major order
