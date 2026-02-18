use axum::{Router, routing::get};
use bluer::{AdapterEvent, Device};
use futures::StreamExt as _;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::time::{Duration, sleep, timeout};
use uuid::Uuid;

const DEVICE_NAME: &str = "Voyager";
const COMPANY_ID: u16 = 0xFFFF;
const DRIFT_THRESHOLD_SECS: u32 = 180; // 3 minutes
const METRICS_PORT: u16 = 9777;

const TIME_SERVICE_UUID: Uuid = Uuid::from_u128(0xa2d7a6e0_5e7b_4f1c_8a3d_b2c9f0e1d4a8);
const TIME_CHAR_UUID: Uuid = Uuid::from_u128(0xa2d7a6e1_5e7b_4f1c_8a3d_b2c9f0e1d4a8);

struct Metrics {
    battery_mv: AtomicU64,
    drift_secs: AtomicI64,
    device_time: AtomicU64,
    last_seen: AtomicU64,
    syncs_total: AtomicU64,
    rssi: AtomicI64,
}

impl Metrics {
    fn new() -> Self {
        Self {
            battery_mv: AtomicU64::new(0),
            drift_secs: AtomicI64::new(0),
            device_time: AtomicU64::new(0),
            last_seen: AtomicU64::new(0),
            syncs_total: AtomicU64::new(0),
            rssi: AtomicI64::new(0),
        }
    }

    fn render(&self) -> String {
        let mut s = String::with_capacity(512);
        let _ = writeln!(s, "# HELP voyager_battery_mv Battery voltage in millivolts from VDDH.");
        let _ = writeln!(s, "# TYPE voyager_battery_mv gauge");
        let _ = writeln!(s, "voyager_battery_mv {}", self.battery_mv.load(Ordering::Relaxed));
        let _ = writeln!(s, "# HELP voyager_drift_seconds Clock drift in seconds (host - device).");
        let _ = writeln!(s, "# TYPE voyager_drift_seconds gauge");
        let _ = writeln!(s, "voyager_drift_seconds {}", self.drift_secs.load(Ordering::Relaxed));
        let _ = writeln!(s, "# HELP voyager_device_time Device reported unix timestamp.");
        let _ = writeln!(s, "# TYPE voyager_device_time gauge");
        let _ = writeln!(s, "voyager_device_time {}", self.device_time.load(Ordering::Relaxed));
        let _ = writeln!(s, "# HELP voyager_last_seen_timestamp Unix timestamp of last advertisement seen.");
        let _ = writeln!(s, "# TYPE voyager_last_seen_timestamp gauge");
        let _ = writeln!(s, "voyager_last_seen_timestamp {}", self.last_seen.load(Ordering::Relaxed));
        let _ = writeln!(s, "# HELP voyager_syncs_total Number of time sync operations performed.");
        let _ = writeln!(s, "# TYPE voyager_syncs_total counter");
        let _ = writeln!(s, "voyager_syncs_total {}", self.syncs_total.load(Ordering::Relaxed));
        let _ = writeln!(s, "# HELP voyager_rssi_dbm RSSI of last received advertisement.");
        let _ = writeln!(s, "# TYPE voyager_rssi_dbm gauge");
        let _ = writeln!(s, "voyager_rssi_dbm {}", self.rssi.load(Ordering::Relaxed));
        s
    }
}

fn now_unix() -> u32 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs() as u32
}

/// Parse manufacturer data payload: [u32 LE time, u16 LE battery_mv]
fn parse_adv_payload(data: &[u8]) -> Option<(u32, u16)> {
    if data.len() < 6 {
        return None;
    }
    let ts = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
    let bat_mv = u16::from_le_bytes([data[4], data[5]]);
    Some((ts, bat_mv))
}

/// Check device manufacturer data, return parsed (device_time, battery_mv) if it's Voyager
async fn check_voyager(device: &Device) -> Option<(u32, u16)> {
    let name = device.name().await.ok().flatten()?;
    if name != DEVICE_NAME {
        return None;
    }
    let mfr = device.manufacturer_data().await.ok()??;
    let data = mfr.get(&COMPANY_ID)?;
    parse_adv_payload(data)
}

/// Connect to device, write current time to the GATT characteristic, disconnect
async fn sync_time(device: &Device) -> bluer::Result<()> {
    println!("  Connecting...");
    device.connect().await?;

    // Wait for services to be resolved (up to 10s)
    for _ in 0..50 {
        if device.is_services_resolved().await? {
            break;
        }
        sleep(Duration::from_millis(200)).await;
    }

    if !device.is_services_resolved().await? {
        eprintln!("  Services not resolved, disconnecting");
        device.disconnect().await?;
        return Ok(());
    }

    // Find the time characteristic
    let mut found = false;
    for service in device.services().await? {
        if service.uuid().await? != TIME_SERVICE_UUID {
            continue;
        }
        for ch in service.characteristics().await? {
            if ch.uuid().await? != TIME_CHAR_UUID {
                continue;
            }
            let ts = now_unix();
            ch.write(&ts.to_le_bytes()).await?;
            println!("  Time set to {ts}");
            found = true;
            break;
        }
        break;
    }

    if !found {
        eprintln!("  Time characteristic not found");
    }

    device.disconnect().await?;
    println!("  Disconnected");
    Ok(())
}

#[tokio::main]
async fn main() -> bluer::Result<()> {
    let metrics = Arc::new(Metrics::new());

    // Prometheus metrics server
    let metrics_ref = Arc::clone(&metrics);
    tokio::spawn(async move {
        let app = Router::new().route(
            "/metrics",
            get(move || {
                let m = Arc::clone(&metrics_ref);
                async move { m.render() }
            }),
        );
        let listener = tokio::net::TcpListener::bind(("0.0.0.0", METRICS_PORT))
            .await
            .expect("failed to bind metrics port");
        println!("Prometheus metrics at http://0.0.0.0:{METRICS_PORT}/metrics");
        axum::serve(listener, app).await.unwrap();
    });

    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;
    adapter.set_powered(true).await?;
    println!("Using adapter {} ({})", adapter.name(), adapter.address().await?);

    loop {
        println!("Scanning for {DEVICE_NAME}...");

        let discover = adapter.discover_devices().await?;
        tokio::pin!(discover);

        let scan_result = timeout(Duration::from_secs(60), async {
            while let Some(evt) = discover.next().await {
                if let AdapterEvent::DeviceAdded(addr) = evt {
                    let device = adapter.device(addr)?;
                    if let Some((dev_time, bat_mv)) = check_voyager(&device).await {
                        let host_time = now_unix();
                        let drift = host_time as i64 - dev_time as i64;

                        // Update metrics
                        metrics.battery_mv.store(bat_mv as u64, Ordering::Relaxed);
                        metrics.drift_secs.store(drift, Ordering::Relaxed);
                        metrics.device_time.store(dev_time as u64, Ordering::Relaxed);
                        metrics.last_seen.store(host_time as u64, Ordering::Relaxed);
                        if let Ok(Some(rssi)) = device.rssi().await {
                            metrics.rssi.store(rssi as i64, Ordering::Relaxed);
                        }

                        println!(
                            "  Found {DEVICE_NAME}: time={dev_time} bat={bat_mv}mV drift={drift}s"
                        );
                        if drift.unsigned_abs() > DRIFT_THRESHOLD_SECS as u64 {
                            println!("  Drift exceeds {DRIFT_THRESHOLD_SECS}s, syncing...");
                            if let Err(e) = sync_time(&device).await {
                                eprintln!("  Sync failed: {e}");
                            } else {
                                metrics.syncs_total.fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        return bluer::Result::Ok(());
                    }
                }
            }
            Ok(())
        })
        .await;

        // Drop the discovery stream before sleeping
        drop(discover);

        match scan_result {
            Ok(Ok(())) => {}
            Ok(Err(e)) => eprintln!("Scan error: {e}"),
            Err(_) => println!("  Scan timeout, no Voyager found"),
        }

        println!("Sleeping 60s...");
        sleep(Duration::from_secs(60)).await;
    }
}
