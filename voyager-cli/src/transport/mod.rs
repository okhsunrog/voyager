//! Transport abstraction for Voyager host communication.
//!
//! Supports:
//! - BLE (via bluer/NUS)

pub mod ble;

use std::time::Duration;
use tokio::io::{AsyncRead, AsyncWrite};

/// A boxed async transport for ergot communication
pub struct Transport {
    pub reader: Box<dyn AsyncRead + Send + Unpin>,
    pub writer: Box<dyn AsyncWrite + Send + Unpin>,
}

impl Transport {
    /// Connect via BLE to a device with the given name (or first found)
    pub async fn connect_ble(
        device_name: Option<&str>,
        timeout: Duration,
    ) -> Result<Self, TransportError> {
        ble::connect(device_name, timeout).await
    }
}

#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    #[error("BLE error: {0}")]
    Ble(#[from] ble::BleError),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}
