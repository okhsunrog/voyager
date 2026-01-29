# Font Viewer - Voyager Display Simulator

Interactive Voyager display simulator using embedded-graphics-simulator.

## Overview

The font-viewer simulates the actual Voyager display layout on a SSD1306 OLED (128x64 pixels). It shows:
- Distance in Astronomical Units (170.0 AU)
- Signal delay time (23:32:35)
- Horizontally scrolling greeting messages

This provides pixel-perfect preview of how the display will look on actual hardware.

## Features

- **Pixel-perfect simulation** of SSD1306 OLED (128x64 pixels)
- **Horizontal scrolling** for long greeting messages
- **Interactive controls** to adjust distance and switch messages
- **Auto-scrolling greetings** from greetings.txt
- **Signal delay calculation** (distance × 8.3 min/AU)
- 8x scale for easy viewing
- White-on-black rendering (matching OLED)
- Only supports 16x16 font (as designed for Voyager display)

## Usage

### View the firmware font

```bash
just view-font
```

This displays the font from `firmware/assets/font.bin` in a simulator window.

### View a custom font

```bash
just view-font path/to/font.bin
# or directly:
cargo run -p font-viewer -- path/to/font.bin
```

### Preview during development

```bash
just view-font-dev
```

This displays the font from `font-generator/font.bin`, useful when testing font generation.

## Display Layout

```
┌────────────────────────────┐ 128×64
│                            │
│ 170.0AU                    │ Y: 4px  (Distance)
│ 23:32:35                   │ Y: 20px (Signal delay time)
│                            │
│ <scrolling greeting text>  │ Y: 40px (Horizontal scroll)
│                            │
└────────────────────────────┘
```

## Controls

- **ESC / Q** - Exit simulator  
- **SPACE** - Pause/Resume scrolling
- **+/-** - Adjust distance (affects signal delay time)
- **↑/↓** - Adjust scroll speed (1-20 pixels/frame)
- **HOME** - Reset scroll to beginning

## Scrolling Behavior

- **Continuous chain**: All greetings are joined into one long text with " • " separators
- **Seamless loop**: When the end scrolls off screen, it loops back to the beginning
- **Smooth animation**: Scrolls at 2 pixels/frame by default (30 FPS)
- **No interruptions**: Text flows continuously like a news ticker

Example: `"Paz e felicidade • Здравствуйте! • สวัสดี • Hello World • ..."`

## Display Simulation

- **Display size**: 128x64 pixels (matches SSD1306)
- **Scale**: 8x (1024x512 window for visibility)
- **Color scheme**: White on black (OLED theme)
- **Font spacing**: Matches actual firmware rendering

## Requirements

- SDL2 library (for embedded-graphics-simulator)
  - Ubuntu/Debian: `sudo apt install libsdl2-dev`
  - Fedora: `sudo dnf install SDL2-devel`
  - macOS: `brew install sdl2`
  - Windows: SDL2 development libraries

## Integration with Firmware

The viewer uses the same font rendering code (`font_generator` crate) as the firmware, ensuring pixel-perfect preview of how text will appear on the device.
