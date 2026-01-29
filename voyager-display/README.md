# Voyager Display Library

Shared `no_std` display rendering library for Voyager firmware and simulator.

## Overview

This library provides the display layout and rendering logic used by both the actual firmware and the desktop simulator. It ensures pixel-perfect consistency between what you see in the simulator and what appears on the device.

## Display Layout (128x64 OLED)

```
┌────────────────────────────┐ 128×64
│                            │
│ 170.0234 AU                │ Y: 4-20px  (Font 16×16)
│ ☉ 23:32:35                 │ Y: 20-36px (Font 16×16)
│                            │ Y: 38px    (separator)
│ Hello from the children... │ Y: 40-56px (Font 16×16)
│                            │
└────────────────────────────┘
```

### Layout Details:
- **Line 1 (Y: 4px)**: Distance in Astronomical Units (AU)
- **Line 2 (Y: 20px)**: Signal delay time (☉ symbol + HH:MM:SS)
- **Line 3 (Y: 40px)**: Greeting message (scrolling)

## Features

- `no_std` compatible for embedded use
- Shares rendering code between firmware and simulator
- Automatic floating-point formatting for distance
- Time display with sun symbol (☉)
- Message scrolling support

## Usage

### In Simulator

```rust
use voyager_display::VoyagerDisplay;

let display_state = VoyagerDisplay::new(
    170.0234,           // Distance in AU
    "23:32:35",         // Signal delay time
    "Hello from Earth" // Greeting message
);

display_state.render(&mut display, &font)?;
```

### In Firmware

```rust
use voyager_display::VoyagerDisplay;

// Same API as simulator!
let state = VoyagerDisplay::new(distance_au, &time_str, &greeting);
state.render(&mut display, &font)?;
```

## Dependencies

- `embedded-graphics` - Display abstraction
- `heapless` - `no_std` String formatting
- `font_generator` - Bitmap font rendering

## Signal Delay Calculation

The time shown represents the one-way light signal delay from Voyager's current distance:
- Distance in AU × 8.3 minutes/AU = signal delay
- Example: 170 AU × 8.3 min/AU ≈ 23 hours 32 minutes

## Integration

This crate is used by:
- **firmware** - Actual device display rendering
- **font-viewer** - Desktop simulator for testing

Both use identical rendering code, ensuring WYSIWYG (What You See Is What You Get).
