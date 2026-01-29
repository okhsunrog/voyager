//! Example build.rs for embedded projects.
//!
//! Copy this file to your embedded project as `build.rs` and adjust the path
//! and font dimensions as needed.
//!
//! This validates the font file at compile time, ensuring:
//! 1. The file exists and is readable
//! 2. The file has valid FNT2 format
//! 3. The font dimensions match your const generics (e.g., 16x16)
//!
//! If validation fails, compilation will fail with a descriptive error message.

// In your embedded project's build.rs:

fn main() {
    // Re-run build script if font file changes
    println!("cargo:rerun-if-changed=assets/font.bin");

    // Validate font at build time
    // Change the const generics to match your font size
    font_generator::validate_font_file::<16, 16>("assets/font.bin").expect(
        "Font validation failed! Check that:\n\
                 1. assets/font.bin exists\n\
                 2. It was generated with --size 16 (or 16x16)\n\
                 3. The file is not corrupted",
    );

    println!("cargo:warning=Font validated successfully!");
}

// Alternative: If you want to also generate a Rust file with the font data:
//
// fn main() {
//     use std::env;
//     use std::fs;
//     use std::path::Path;
//
//     println!("cargo:rerun-if-changed=assets/font.bin");
//
//     // Validate
//     let info = font_generator::validate_font_file::<16, 16>("assets/font.bin")
//         .expect("Font validation failed");
//
//     // Generate a constants file
//     let out_dir = env::var("OUT_DIR").unwrap();
//     let dest_path = Path::new(&out_dir).join("font_info.rs");
//
//     let contents = format!(
//         "/// Font width in pixels\n\
//          pub const FONT_WIDTH: usize = {};\n\
//          /// Font height in pixels\n\
//          pub const FONT_HEIGHT: usize = {};\n\
//          /// Number of glyphs in the font\n\
//          pub const GLYPH_COUNT: u32 = {};\n",
//         info.width, info.height, info.glyph_count
//     );
//
//     fs::write(&dest_path, contents).unwrap();
//
//     println!("cargo:warning=Generated font_info.rs with {} glyphs", info.glyph_count);
// }
//
// Then in your main code:
// include!(concat!(env!("OUT_DIR"), "/font_info.rs"));
