//! BLE transport via bluer (BlueZ)
//!
//! Implements Nordic UART Service (NUS) communication for ergot over BLE.

use super::{Transport, TransportError};
use bluer::{AdapterEvent, Device, gatt::remote::Characteristic};
use futures::StreamExt;
use std::time::Duration;
use uuid::Uuid;

/// Nordic UART Service UUID
const NUS_SERVICE_UUID: Uuid = Uuid::from_u128(0x6E400001_B5A3_F393_E0A9_E50E24DCCA9E);
/// NUS TX Characteristic UUID (write - host to device)
const NUS_TX_CHAR_UUID: Uuid = Uuid::from_u128(0x6E400002_B5A3_F393_E0A9_E50E24DCCA9E);
/// NUS RX Characteristic UUID (notify - device to host)
const NUS_RX_CHAR_UUID: Uuid = Uuid::from_u128(0x6E400003_B5A3_F393_E0A9_E50E24DCCA9E);

#[derive(Debug, thiserror::Error)]
pub enum BleError {
    #[error("BlueZ error: {0}")]
    Bluer(#[from] bluer::Error),

    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    #[error("Characteristic not found: {0}")]
    CharacteristicNotFound(String),

    #[error("Connection failed: {0}")]
    ConnectionFailed(String),
}

/// Connect to a BLE device via Nordic UART Service
pub async fn connect(
    device_name: Option<&str>,
    timeout: Duration,
) -> Result<Transport, TransportError> {
    let session = bluer::Session::new().await.map_err(BleError::from)?;
    let adapter = session.default_adapter().await.map_err(BleError::from)?;

    // Ensure adapter is powered on
    if !adapter.is_powered().await.map_err(BleError::from)? {
        adapter.set_powered(true).await.map_err(BleError::from)?;
    }

    log::info!(
        "Scanning for BLE devices on adapter {} ({})",
        adapter.name(),
        adapter.address().await.map_err(BleError::from)?
    );

    // Find and connect to device
    let device = find_device(&adapter, device_name, timeout).await?;

    log::info!("Found device: {}", device.address());

    // Connect if not already connected
    if !device.is_connected().await.map_err(BleError::from)? {
        log::info!("Connecting to device...");
        let mut retries = 3;
        loop {
            match device.connect().await {
                Ok(()) => break,
                Err(e) if retries > 0 => {
                    log::warn!("Connection attempt failed: {}, retrying...", e);
                    retries -= 1;
                    tokio::time::sleep(Duration::from_millis(500)).await;
                }
                Err(e) => return Err(BleError::ConnectionFailed(e.to_string()).into()),
            }
        }
    }

    log::info!("Connected to device");

    // Wait for services to be resolved
    tokio::time::sleep(Duration::from_millis(500)).await;

    // Find NUS service and characteristics
    let (tx_char, rx_char) = find_nus_characteristics(&device).await?;

    log::info!("Found NUS service, setting up I/O...");

    // Get write IO for TX characteristic
    let write_io = tx_char.write_io().await.map_err(BleError::from)?;

    // Get notify IO for RX characteristic
    let notify_io = rx_char.notify_io().await.map_err(BleError::from)?;

    log::info!(
        "BLE transport ready (RX MTU: {}, TX MTU: {})",
        notify_io.mtu(),
        write_io.mtu()
    );

    Ok(Transport {
        reader: Box::new(notify_io),
        writer: Box::new(write_io),
    })
}

async fn find_device(
    adapter: &bluer::Adapter,
    name_filter: Option<&str>,
    timeout: Duration,
) -> Result<Device, BleError> {
    let discover = adapter.discover_devices().await?;
    futures::pin_mut!(discover);

    let deadline = tokio::time::Instant::now() + timeout;

    while let Ok(Some(evt)) = tokio::time::timeout_at(deadline, discover.next()).await {
        if let AdapterEvent::DeviceAdded(addr) = evt {
            let device = adapter.device(addr)?;

            // Check if device has NUS service
            let uuids = device.uuids().await?.unwrap_or_default();
            if !uuids.contains(&NUS_SERVICE_UUID) {
                continue;
            }

            // If name filter specified, check it
            if let Some(filter) = name_filter {
                let name = device.name().await?.unwrap_or_default();
                if !name.to_lowercase().contains(&filter.to_lowercase()) {
                    log::debug!("Skipping device {} (name: {})", addr, name);
                    continue;
                }
            }

            let name = device.name().await?.unwrap_or_else(|| "Unknown".into());
            log::info!("Found NUS device: {} ({})", name, addr);

            return Ok(device);
        }
    }

    Err(BleError::DeviceNotFound(
        name_filter
            .map(|n| format!("No device matching '{}' with NUS service found", n))
            .unwrap_or_else(|| "No device with NUS service found".into()),
    ))
}

async fn find_nus_characteristics(
    device: &Device,
) -> Result<(Characteristic, Characteristic), BleError> {
    let mut tx_char = None;
    let mut rx_char = None;

    for service in device.services().await? {
        let uuid = service.uuid().await?;
        if uuid != NUS_SERVICE_UUID {
            continue;
        }

        log::debug!("Found NUS service");

        for char in service.characteristics().await? {
            let char_uuid = char.uuid().await?;

            if char_uuid == NUS_TX_CHAR_UUID {
                log::debug!("Found NUS TX characteristic");
                tx_char = Some(char);
            } else if char_uuid == NUS_RX_CHAR_UUID {
                log::debug!("Found NUS RX characteristic");
                rx_char = Some(char);
            }
        }

        break;
    }

    let tx = tx_char.ok_or_else(|| BleError::CharacteristicNotFound("NUS TX".into()))?;
    let rx = rx_char.ok_or_else(|| BleError::CharacteristicNotFound("NUS RX".into()))?;

    Ok((tx, rx))
}

/// Scan for available BLE devices with NUS service
pub async fn scan_devices(timeout: Duration) -> Result<Vec<ScannedDevice>, BleError> {
    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;

    if !adapter.is_powered().await? {
        adapter.set_powered(true).await?;
    }

    let mut devices = Vec::new();
    let discover = adapter.discover_devices().await?;
    futures::pin_mut!(discover);

    let deadline = tokio::time::Instant::now() + timeout;

    while let Ok(Some(evt)) = tokio::time::timeout_at(deadline, discover.next()).await {
        if let AdapterEvent::DeviceAdded(addr) = evt {
            let device = adapter.device(addr)?;

            let uuids = device.uuids().await?.unwrap_or_default();
            if !uuids.contains(&NUS_SERVICE_UUID) {
                continue;
            }

            let name = device.name().await?.unwrap_or_else(|| "Unknown".into());
            let rssi = device.rssi().await?.unwrap_or(0);

            devices.push(ScannedDevice {
                address: addr.to_string(),
                name,
                rssi,
            });
        }
    }

    Ok(devices)
}

#[derive(Debug, Clone)]
pub struct ScannedDevice {
    pub address: String,
    pub name: String,
    pub rssi: i16,
}
