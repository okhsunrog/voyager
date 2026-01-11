//! DFU flashing library for Adafruit nRF52 Bootloader
//!
//! This crate implements the serial DFU protocol used by the Adafruit nRF52 Bootloader.
//! It supports flashing firmware to nRF52 devices over USB CDC serial.

mod elf;
mod protocol;
mod slip;

pub use elf::elf_to_bin;
pub use protocol::{ADAFRUIT_VID, DfuConnection, DfuError};

use std::time::Duration;
use tokio_serial::SerialPortType;

/// Find serial port with Adafruit VID
pub fn find_adafruit_serial_port() -> Option<String> {
    let ports = tokio_serial::available_ports().ok()?;

    for port in ports {
        if let SerialPortType::UsbPort(usb_info) = &port.port_type
            && usb_info.vid == ADAFRUIT_VID
        {
            log::info!("Found Adafruit device at {}", port.port_name);
            return Some(port.port_name);
        }
    }

    None
}

/// Wait for a serial port with Adafruit VID to appear
pub async fn wait_for_dfu_device(timeout: Duration) -> Result<String, DfuError> {
    let start = std::time::Instant::now();

    loop {
        if let Some(port) = find_adafruit_serial_port() {
            // Give the port a moment to initialize
            tokio::time::sleep(Duration::from_millis(100)).await;
            return Ok(port);
        }

        if start.elapsed() > timeout {
            return Err(DfuError::Timeout("waiting for DFU device".into()));
        }

        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}
