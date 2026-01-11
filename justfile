# Voyager project commands

# Default recipe - show available commands
default:
    @just --list

# Build the firmware
build:
    cargo build -p firmware --release

# Flash firmware via DFU and monitor logs
flash: build
    cargo run -p voyager-cli --release -- {{target_dir}}/thumbv7em-none-eabihf/release/firmware

# Flash firmware without monitoring
flash-only: build
    cargo run -p voyager-cli --release -- {{target_dir}}/thumbv7em-none-eabihf/release/firmware flash --no-monitor

# Stream logs from device (USB)
logs:
    cargo run -p voyager-cli --release -- logs

# Stream logs from device (BLE)
logs-ble device="":
    cargo run -p voyager-cli --release -- --transport ble {{if device != "" { "--ble-device " + device } else { "" }}} logs

# Set device time to current time
set-time:
    cargo run -p voyager-cli --release -- set-time

# Get device time
get-time:
    cargo run -p voyager-cli --release -- get-time

# Reboot device into DFU mode
reboot-dfu:
    cargo run -p voyager-cli --release -- reboot-dfu

# Scan for BLE devices with NUS service
scan-ble duration="5":
    cargo run -p voyager-cli --release -- scan-ble --duration {{duration}}

# Build voyager-cli
build-cli:
    cargo build -p voyager-cli --release

# Run clippy on all crates
clippy:
    cargo clippy --all

# Format all code
fmt:
    cargo fmt --all

# Check formatting
fmt-check:
    cargo fmt --all -- --check

# Clean build artifacts
clean:
    cargo clean

# Private: get target directory
target_dir := `cargo metadata --format-version 1 | jq -r .target_directory`
