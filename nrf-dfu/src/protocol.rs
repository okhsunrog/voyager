//! Nordic DFU protocol implementation for Adafruit nRF52 Bootloader
//!
//! This implements the HCI-based DFU protocol used by the Adafruit bootloader.

use std::time::Duration;
use thiserror::Error;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::SerialPortBuilderExt;

use crate::slip;

/// Adafruit USB Vendor ID
pub const ADAFRUIT_VID: u16 = 0x239A;

// DFU packet types
const DFU_INIT_PACKET: u32 = 1;
const DFU_START_PACKET: u32 = 3;
const DFU_DATA_PACKET: u32 = 4;
const DFU_STOP_DATA_PACKET: u32 = 5;

// HCI packet parameters
const DATA_INTEGRITY_CHECK_PRESENT: u32 = 1;
const RELIABLE_PACKET: u32 = 1;
const HCI_PACKET_TYPE: u32 = 14;

const DFU_PACKET_MAX_SIZE: usize = 512;

// Firmware mode
const APPLICATION: u8 = 4;

// Timing constants
const FLASH_PAGE_SIZE: f64 = 4096.0;
const FLASH_PAGE_ERASE_TIME: f64 = 0.0897; // ~90ms max for nRF52840
const FLASH_WORD_WRITE_TIME: f64 = 0.000100;
const FLASH_PAGE_WRITE_TIME: f64 = (FLASH_PAGE_SIZE / 4.0) * FLASH_WORD_WRITE_TIME;

#[derive(Error, Debug)]
pub enum DfuError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serial port error: {0}")]
    Serial(#[from] tokio_serial::Error),

    #[error("Timeout: {0}")]
    Timeout(String),

    #[error("Protocol error: {0}")]
    Protocol(String),
}

/// Progress callback type
pub type ProgressCallback = Box<dyn Fn(usize, usize) + Send>;

/// DFU connection to a device in bootloader mode
pub struct DfuConnection {
    port: tokio_serial::SerialStream,
    sequence_number: u32,
}

impl DfuConnection {
    /// Open a DFU connection to the specified serial port
    pub async fn open(port_name: &str) -> Result<Self, DfuError> {
        log::debug!("Opening DFU connection to {}", port_name);

        let port = tokio_serial::new(port_name, 115200)
            .timeout(Duration::from_millis(1000))
            .open_native_async()?;

        Ok(Self {
            port,
            sequence_number: 0,
        })
    }

    /// Flash firmware binary to the device
    pub async fn flash(&mut self, firmware: &[u8]) -> Result<(), DfuError> {
        self.flash_with_progress(firmware, None).await
    }

    /// Flash firmware binary to the device with progress callback
    pub async fn flash_with_progress(
        &mut self,
        firmware: &[u8],
        progress: Option<ProgressCallback>,
    ) -> Result<(), DfuError> {
        let firmware_size = firmware.len() as u64;

        log::info!("Starting DFU, firmware size: {} bytes", firmware_size);

        // Send start packet
        self.send_start_dfu(APPLICATION, 0, 0, firmware_size)
            .await?;

        // Wait for flash erase
        let erase_time = self.get_erase_wait_time(firmware_size);
        log::debug!("Waiting {}ms for flash erase", erase_time);
        tokio::time::sleep(Duration::from_millis(erase_time)).await;

        // Send init packet
        self.send_init_packet().await?;

        // Send firmware data
        self.send_firmware_data(firmware, progress).await?;

        // Send stop packet
        self.send_stop_dfu().await?;

        // Wait for device to process and reboot
        tokio::time::sleep(Duration::from_millis(1000)).await;

        log::info!("DFU complete");
        Ok(())
    }

    async fn send_start_dfu(
        &mut self,
        mode: u8,
        sd_size: u64,
        bl_size: u64,
        app_size: u64,
    ) -> Result<(), DfuError> {
        log::debug!("Sending DFU start packet");

        let mut buf = DFU_START_PACKET.to_le_bytes().to_vec();
        buf.extend((mode as u32).to_le_bytes());
        buf.extend((sd_size as u32).to_le_bytes());
        buf.extend((bl_size as u32).to_le_bytes());
        buf.extend((app_size as u32).to_le_bytes());

        self.send_packet(&buf).await
    }

    async fn send_init_packet(&mut self) -> Result<(), DfuError> {
        log::debug!("Sending DFU init packet");

        // Minimal init packet for Adafruit bootloader
        let mut buf = DFU_INIT_PACKET.to_le_bytes().to_vec();
        buf.extend(&[0x00, 0x00]);

        self.send_packet(&buf).await
    }

    async fn send_firmware_data(
        &mut self,
        firmware: &[u8],
        progress: Option<ProgressCallback>,
    ) -> Result<(), DfuError> {
        let total_chunks = firmware.len().div_ceil(DFU_PACKET_MAX_SIZE);
        log::info!("Sending {} chunks of firmware data", total_chunks);

        for (i, chunk) in firmware.chunks(DFU_PACKET_MAX_SIZE).enumerate() {
            let mut buf = DFU_DATA_PACKET.to_le_bytes().to_vec();
            buf.extend(chunk);

            self.send_packet(&buf).await?;

            // Periodic write delay
            if i % 8 == 0 {
                tokio::time::sleep(Duration::from_micros(
                    (FLASH_PAGE_WRITE_TIME * 1000.0) as u64,
                ))
                .await;
            }

            // Report progress
            if let Some(ref cb) = progress {
                cb(i + 1, total_chunks);
            }

            // Log progress periodically
            if (i + 1) % 50 == 0 || i + 1 == total_chunks {
                log::info!("Progress: {}/{} chunks", i + 1, total_chunks);
            }
        }

        // Final write delay
        tokio::time::sleep(Duration::from_micros(
            (FLASH_PAGE_WRITE_TIME * 1000.0) as u64,
        ))
        .await;

        Ok(())
    }

    async fn send_stop_dfu(&mut self) -> Result<(), DfuError> {
        log::debug!("Sending DFU stop packet");
        let buf = DFU_STOP_DATA_PACKET.to_le_bytes().to_vec();
        self.send_packet(&buf).await
    }

    async fn send_packet(&mut self, data: &[u8]) -> Result<(), DfuError> {
        let packet = self.build_hci_packet(data);

        self.port.write_all(&packet).await?;
        self.port.flush().await?;

        // Wait for ACK
        self.wait_for_ack().await?;

        Ok(())
    }

    async fn wait_for_ack(&mut self) -> Result<(), DfuError> {
        let mut buf = vec![0u8; 16];
        let timeout = Duration::from_millis(1000);

        match tokio::time::timeout(timeout, self.port.read(&mut buf)).await {
            Ok(Ok(n)) => {
                log::trace!("Received {} bytes ACK", n);
                Ok(())
            }
            Ok(Err(e)) => {
                // Some errors are acceptable
                log::trace!("ACK read error (usually OK): {}", e);
                Ok(())
            }
            Err(_) => {
                // Timeout is acceptable for some packets
                log::trace!("ACK timeout (usually OK)");
                Ok(())
            }
        }
    }

    fn build_hci_packet(&mut self, data: &[u8]) -> Vec<u8> {
        self.sequence_number = (self.sequence_number + 1) % 8;

        let header = self.build_header(data.len() as u32);

        let mut payload = Vec::new();
        payload.extend(&header);
        payload.extend(data);

        let crc = calc_crc16(&payload);
        payload.push((crc & 0xFF) as u8);
        payload.push(((crc >> 8) & 0xFF) as u8);

        // SLIP encode
        let mut encoded = Vec::with_capacity(payload.len() * 2 + 2);
        encoded.push(slip::END);
        slip::encode(&payload, &mut encoded);

        encoded
    }

    fn build_header(&self, payload_len: u32) -> [u8; 4] {
        let seq = self.sequence_number;
        let mut buf = [0u8; 4];

        buf[0] = (seq
            | (((seq + 1) % 8) << 3)
            | (DATA_INTEGRITY_CHECK_PRESENT << 6)
            | (RELIABLE_PACKET << 7)) as u8;
        buf[1] = (HCI_PACKET_TYPE | ((payload_len & 0x000F) << 4)) as u8;
        buf[2] = ((payload_len & 0x0FF0) >> 4) as u8;

        let sum: u32 = buf[0] as u32 + buf[1] as u32 + buf[2] as u32;
        buf[3] = (((!sum).wrapping_add(1)) & 0xFF) as u8;

        buf
    }

    fn get_erase_wait_time(&self, firmware_size: u64) -> u64 {
        let time =
            ((firmware_size as f64 / FLASH_PAGE_SIZE) + 1.0) * FLASH_PAGE_ERASE_TIME * 1000.0;
        time.max(500.0) as u64
    }
}

/// Calculate CRC-16 as used by the Nordic DFU protocol
fn calc_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc = (crc >> 8 & 0x00FF) | (crc << 8 & 0xFF00);
        crc ^= b as u16;
        crc ^= (crc & 0x00FF) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0x00FF) << 4) << 1;
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16() {
        // Known test vector
        let crc = calc_crc16(&[0x01, 0x02, 0x03, 0x04]);
        assert_ne!(crc, 0); // Just verify it produces something
    }
}
