# Voyager

Embedded firmware and tools for an nRF52840-based Voyager 1 distance tracker. Displays real-time distance from the Sun and light signal delay on an SSD1306 OLED, with scrolling multilingual greetings. Communicates over BLE (GATT time service) and USB CDC serial.

Tested on nice!nano v2 and Seeed XIAO nRF52840 Sense with Adafruit nRF52 bootloader.

## Getting started

The `prebuilt/` directory contains pre-built UF2/hex files for board setup. Pick the section that matches your board's current state.

### Board without a bootloader (bare nRF52840)

Flash the Adafruit bootloader using a debug probe (J-Link, ST-Link, etc.):

```sh
nrfjprog --program prebuilt/nice_nano_bootloader-0.9.0_nosd.hex --chiperase -f nrf52 --reset
```

After flashing, the board will boot into the Adafruit bootloader (LED pulses). The SoftDevice is not included, so `memory.x` is already correct (`ORIGIN = 0x1000`) — skip straight to [Building and flashing](#building-and-flashing).

### Board with bootloader and S140 v6.1.1 (nice!nano v2)

1. Double-tap reset to enter bootloader mode
2. Copy `prebuilt/erase-sd-s140-v6.1.1.uf2` to the USB drive that appears
3. Board erases the SoftDevice and reboots into bootloader
4. Verify: mount the UF2 drive and check `INFO_UF2.TXT` shows `SoftDevice: not found`

### Board with bootloader and S140 v7.3.0 (Seeed XIAO)

1. Double-tap reset to enter bootloader mode
2. Copy `prebuilt/erase-sd-s140-v7.3.0.uf2` to the USB drive that appears
3. Board erases the SoftDevice and reboots into bootloader
4. Verify: mount the UF2 drive and check `INFO_UF2.TXT` shows `SoftDevice: not found`

### Identifying your SoftDevice version

Enter bootloader mode (double-tap reset), mount the UF2 drive, and read `INFO_UF2.TXT`. The `SoftDevice:` line tells you which version is installed:

| SoftDevice | Boards |
|---|---|
| S140 v6.1.1 | nice!nano v2 (bootloader v0.6.0) |
| S140 v7.3.0 | Seeed XIAO nRF52840 Sense (bootloader v0.6.1) |

## Prerequisites

- Rust toolchain with `thumbv7em-none-eabihf` target
- `cargo-binutils` (`cargo install cargo-binutils`)
- `cargo-hex-to-uf2` (`cargo install cargo-hex-to-uf2`)
- `adafruit-nrfutil` (`uv tool install adafruit-nrfutil`)
- `just` command runner
- `python3` with `pyserial` (for 1200-baud touch in `just dfu`)

## Building and flashing

Build the firmware:

```sh
just build
```

### UF2 (manual)

1. Double-tap reset to enter bootloader mode
2. Build the UF2 file:
   ```sh
   just uf2
   ```
3. Copy `firmware/target/voyager.uf2` to the USB drive

### DFU (automated, app already running)

Triggers a 1200-baud reset to enter bootloader, then flashes via serial DFU:

```sh
just dfu
```

### DFU (board already in bootloader mode)

```sh
just dfu-bl
```

### Port auto-detection

The `justfile` auto-detects serial ports by USB product name. You can override manually:

```sh
just APP_PORT=/dev/ttyACM0 dfu
just BL_PORT=/dev/ttyACM0 dfu-bl
```

## Memory layout

The flash origin in `firmware/memory.x` depends on whether the SoftDevice is present:

| Config | Flash origin | Flash length | Notes |
|---|---|---|---|
| No SoftDevice | `0x1000` | 0xF3000 (972K) | Recommended — erase SD first (see [Getting started](#getting-started)) |
| S140 v7.3.0 | `0x27000` | 0xCD000 (820K) | Seeed XIAO stock (bootloader v0.6.1) |
| S140 v6.1.1 | `0x26000` | 0xCE000 (824K) | nice!nano v2 stock (bootloader v0.6.0) |

RAM starts at `0x20000000` (256K) on all nRF52840 boards.

If the flash origin doesn't match the board's actual configuration, the firmware will crash at startup and the board will fall back to bootloader mode. Check your board's config by entering bootloader mode (double-tap reset), mounting the UF2 drive, and reading `INFO_UF2.TXT`.

## Reading logs

```sh
tio /dev/ttyACM0
```

Or use the stable by-id path:

```sh
tio /dev/serial/by-id/usb-Voyager_Voyager_CDC_12345678-if00
```

## Workspace structure

The project is a Cargo workspace with the firmware excluded (different target):

| Crate | Description |
|---|---|
| `firmware/` | nRF52840 firmware (no_std, `thumbv7em-none-eabihf`) |
| `voyager-display/` | Shared display rendering library (no_std) — distance calculations, text formatting, scrolling bitmap |
| `font-viewer/` | Desktop simulator using `embedded-graphics-simulator` — previews the full SSD1306 display |
| `greeting-renderer/` | Renders multilingual greetings text to a 1-bit BMP for firmware use |

## Hardware

- **MCU**: nRF52840 (ARM Cortex-M4F)
- **Display**: SSD1306 128x64 OLED via I2C (P0.17 SDA, P0.20 SCL, 400 kHz)
- **LED**: P0.15 (active low), blinks 200ms on / 1800ms off
- **VCC enable**: P0.13 (nice!nano specific, set HIGH for 3.3V)
- **BLE**: nRF SoftDevice Controller with Trouble host stack, advertises as "Voyager" with GATT time service
- **USB**: CDC ACM for log output + 1200-baud bootloader reset detection

## Key details

- **VTOR**: `cortex-m-rt` `set-vtor` feature is required because the Adafruit bootloader does not set VTOR before jumping to the app
- **Display layout**: top 32px show Voyager 1 distance + signal delay, bottom 32px show scrolling greetings bitmap
- **Time sync**: current time is seeded from compile-time timestamp (`BUILD_TIMESTAMP`), updated via BLE GATT write or internal tick
- **Distance tracking**: fixed-point arithmetic (no FPU), based on NASA JPL Horizons ephemeris data (Jan 1, 2025 reference: 165.6962 AU at 16.88225 km/s)
- **1200-baud reset**: opening the CDC port at 1200 baud triggers a reset into bootloader (GPREGRET=0x57)
