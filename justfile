# Voyager project commands

# App port: auto-detect by USB product name
APP_PORT := `ls /dev/serial/by-id/usb-Voyager_Voyager_CDC_* 2>/dev/null | head -1 || echo ""`
# Bootloader port: auto-detect Adafruit nRF52 bootloader
BL_PORT := `ls /dev/serial/by-id/usb-Seeed_* /dev/serial/by-id/usb-Nice_Keyboards_* /dev/serial/by-id/usb-Adafruit_* 2>/dev/null | head -1 || echo ""`

default: uf2

# Build the firmware
build:
    cd firmware && cargo build --release

# Convert to ihex
objcopy: build
    cd firmware && cargo objcopy --release --bin voyager -- -O ihex target/voyager.hex

# Convert to UF2
uf2: objcopy
    cargo hex-to-uf2 --input-path firmware/target/voyager.hex --output-path firmware/target/voyager.uf2 --family nrf52840

# Create DFU package
dfu-pkg: objcopy
    adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application firmware/target/voyager.hex firmware/target/voyager.zip

# Enter bootloader via 1200-baud touch, then DFU flash
dfu: dfu-pkg
    #!/usr/bin/env bash
    set -euo pipefail
    APP="{{APP_PORT}}"
    if [ -z "$APP" ]; then
        echo "Error: app port not found. Is the firmware running?"
        exit 1
    fi
    # Touch at 1200 baud to trigger bootloader reset
    timeout 3 python3 -c "import serial, time; s = serial.Serial('$APP', 1200); time.sleep(0.1); s.close()" 2>/dev/null || true
    echo "Triggered 1200-baud reset, waiting for bootloader..."
    # Wait for bootloader port to appear (up to 10s)
    BL=""
    for i in $(seq 1 10); do
        BL=$(ls /dev/serial/by-id/usb-Seeed_* /dev/serial/by-id/usb-Nice_Keyboards_* /dev/serial/by-id/usb-Adafruit_* 2>/dev/null | head -1 || true)
        if [ -n "$BL" ]; then
            break
        fi
        sleep 1
    done
    if [ -z "$BL" ]; then
        echo "Error: bootloader port not found"
        exit 1
    fi
    sleep 1
    # DFU on the bootloader's serial port
    adafruit-nrfutil dfu serial --package firmware/target/voyager.zip -p "$BL" -b 115200

# DFU flash when already in bootloader mode
dfu-bl: dfu-pkg
    #!/usr/bin/env bash
    set -euo pipefail
    BL="{{BL_PORT}}"
    if [ -z "$BL" ]; then
        echo "Error: bootloader port not found. Double-tap reset to enter bootloader."
        exit 1
    fi
    adafruit-nrfutil dfu serial --package firmware/target/voyager.zip -p "$BL" -b 115200

# Run clippy
clippy:
    cd firmware && cargo clippy --release

# Format all code
fmt:
    cargo fmt --all

# Check formatting
fmt-check:
    cargo fmt --all -- --check

# Clean build artifacts
clean:
    cargo clean
