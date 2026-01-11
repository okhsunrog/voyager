//! Voyager CLI - Host tool for Voyager firmware
//!
//! Commands:
//! - flash: Flash firmware and monitor logs (can be used as cargo runner)
//! - logs: Stream logs from the device
//! - set-time: Set the device time to current time
//! - get-time: Get the current device time
//! - reboot-dfu: Reboot the device into bootloader mode
//! - scan-ble: Scan for BLE devices with NUS service

mod transport;

use std::{collections::HashSet, path::PathBuf, pin::pin, time::Duration};

use clap::{Parser, Subcommand, ValueEnum};
use cobs_acc::{CobsAccumulator, FeedResult};
use ergot::{
    Address,
    interface_manager::{InterfaceState, profiles::direct_edge::DirectEdge},
    net_stack::ArcNetStack,
    toolkits::nusb_v0_1::{RouterStack, find_new_devices, register_router_interface},
    well_known::ErgotFmtRxTopic,
};
use log::{error, info, warn};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::time::{sleep, timeout};
use voyager_icd::{GetTimeEndpoint, RebootToDfuEndpoint, SetTimeEndpoint};

const MTU: u16 = 1024;
const OUT_BUFFER_SIZE: usize = 4096;

#[derive(Debug, Clone, Copy, Default, ValueEnum)]
enum TransportMode {
    /// USB via ergot (default)
    #[default]
    Usb,
    /// BLE via Nordic UART Service
    Ble,
}

#[derive(Parser)]
#[command(name = "voyager-cli")]
#[command(about = "Host tool for Voyager firmware", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,

    /// ELF file to flash (for use as cargo runner)
    #[arg(value_name = "ELF_FILE")]
    elf_file: Option<PathBuf>,

    /// Transport mode (usb or ble)
    #[arg(short, long, value_enum, default_value_t = TransportMode::Usb)]
    transport: TransportMode,

    /// BLE device name filter (for --transport ble)
    #[arg(long)]
    ble_device: Option<String>,
}

#[derive(Subcommand)]
enum Commands {
    /// Flash firmware to device and monitor logs
    Flash {
        /// Path to the ELF file to flash
        elf_path: PathBuf,

        /// Don't monitor logs after flashing
        #[arg(long)]
        no_monitor: bool,
    },
    /// Stream logs from the device
    Logs,
    /// Set device time to current system time
    SetTime,
    /// Get current device time
    GetTime,
    /// Reboot device into DFU/bootloader mode
    RebootDfu,
    /// Scan for BLE devices with NUS service
    ScanBle {
        /// Scan duration in seconds
        #[arg(short, long, default_value_t = 5)]
        duration: u64,
    },
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let cli = Cli::parse();

    // Handle cargo runner mode (direct ELF file argument)
    if let Some(elf_path) = cli.elf_file {
        return flash_and_monitor(elf_path, false).await;
    }

    // Handle subcommands
    match cli.command {
        Some(Commands::Flash {
            elf_path,
            no_monitor,
        }) => flash_and_monitor(elf_path, no_monitor).await,
        Some(Commands::Logs) => {
            run_with_stack(
                &cli,
                |stack| async move { stream_logs_generic(stack).await },
            )
            .await
        }
        Some(Commands::SetTime) => {
            run_with_stack(&cli, |stack| async move { set_time_generic(stack).await }).await
        }
        Some(Commands::GetTime) => {
            run_with_stack(&cli, |stack| async move { get_time_generic(stack).await }).await
        }
        Some(Commands::RebootDfu) => {
            run_with_stack(&cli, |stack| async move { reboot_dfu_generic(stack).await }).await
        }
        Some(Commands::ScanBle { duration }) => {
            scan_ble_devices(Duration::from_secs(duration)).await
        }
        None => {
            eprintln!("No command specified. Use --help for usage.");
            std::process::exit(1);
        }
    }
}

/// Ergot stack abstraction for USB and BLE
enum ErgotStack {
    Usb(RouterStack),
    Ble(BleStack),
}

type BleStack = ArcNetStack<
    CriticalSectionRawMutex,
    DirectEdge<ergot::interface_manager::interface_impls::tokio_serial_cobs::TokioSerialInterface>,
>;

/// Run a command with the appropriate stack based on transport mode
async fn run_with_stack<F, Fut>(cli: &Cli, f: F) -> Result<(), Box<dyn std::error::Error>>
where
    F: FnOnce(ErgotStack) -> Fut,
    Fut: std::future::Future<Output = Result<(), Box<dyn std::error::Error>>>,
{
    match cli.transport {
        TransportMode::Usb => {
            let stack = connect_usb().await?;
            f(ErgotStack::Usb(stack)).await
        }
        TransportMode::Ble => {
            let stack = connect_ble(cli.ble_device.as_deref()).await?;
            f(ErgotStack::Ble(stack)).await
        }
    }
}

async fn scan_ble_devices(duration: Duration) -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Scanning for BLE devices with NUS service for {} seconds...",
        duration.as_secs()
    );

    let devices = transport::ble::scan_devices(duration).await?;

    if devices.is_empty() {
        println!("No devices found.");
    } else {
        println!("\nFound {} device(s):\n", devices.len());
        for dev in devices {
            println!("  {} ({}) RSSI: {} dBm", dev.name, dev.address, dev.rssi);
        }
    }

    Ok(())
}

async fn connect_usb() -> Result<RouterStack, Box<dyn std::error::Error>> {
    let stack: RouterStack = RouterStack::new();

    info!("Scanning for Voyager USB device...");
    let device = loop {
        let devices = find_new_devices(&HashSet::new()).await;
        if let Some(dev) = devices.into_iter().next() {
            break dev;
        }
        sleep(Duration::from_millis(500)).await;
    };

    info!("Found device: {:?}", device.info);
    let _hdl = register_router_interface(&stack, device, MTU, OUT_BUFFER_SIZE)
        .await
        .map_err(|e| format!("Failed to register interface: {:?}", e))?;

    sleep(Duration::from_millis(100)).await;
    Ok(stack)
}

async fn connect_ble(device_name: Option<&str>) -> Result<BleStack, Box<dyn std::error::Error>> {
    use ergot::interface_manager::profiles::direct_edge::process_frame as ergot_edge_process_frame;
    use ergot::interface_manager::utils::cobs_stream::Sink as ErgotSink;
    use ergot::interface_manager::utils::std::new_std_queue;

    info!("Connecting via BLE...");

    let transport = transport::Transport::connect_ble(device_name, Duration::from_secs(10)).await?;
    let (mut transport_rx, mut transport_tx) = (transport.reader, transport.writer);

    let queue = new_std_queue(4096);

    let stack: BleStack = ArcNetStack::new_with_profile(DirectEdge::new_controller(
        ErgotSink::new_from_handle(queue.clone(), MTU),
        InterfaceState::Active {
            net_id: 1,
            node_id: 1,
        },
    ));

    // Spawn reader task
    tokio::spawn({
        let stack = stack.clone();
        async move {
            let mut buf = vec![0u8; 2048];
            let mut cobs_acc = CobsAccumulator::new_boxslice((MTU as usize) + 64);
            let mut net_id = Some(1u16);
            loop {
                match transport_rx.read(&mut buf).await {
                    Ok(0) => {
                        error!("BLE transport closed");
                        break;
                    }
                    Ok(count) => {
                        let mut window = &mut buf[..count];
                        while !window.is_empty() {
                            window = match cobs_acc.feed_raw(window) {
                                FeedResult::Consumed => break,
                                FeedResult::OverFull(rem) | FeedResult::DecodeError(rem) => rem,
                                FeedResult::Success { data, remaining }
                                | FeedResult::SuccessInput { data, remaining } => {
                                    ergot_edge_process_frame(&mut net_id, data, &stack, ());
                                    remaining
                                }
                            };
                        }
                    }
                    Err(e) => {
                        error!("BLE read error: {:?}", e);
                        break;
                    }
                }
            }
        }
    });

    // Spawn writer task
    tokio::spawn({
        let tx_queue = queue.clone();
        async move {
            let tx_consumer = tx_queue.stream_consumer();
            loop {
                let frame = tx_consumer.wait_read().await;
                let len = frame.len();
                if len == 0 {
                    frame.release(len);
                    continue;
                }

                if let Err(e) = transport_tx.write_all(&frame[..len]).await {
                    error!("BLE write error: {:?}", e);
                    frame.release(len);
                    break;
                }
                frame.release(len);
            }
        }
    });

    info!("BLE ergot stack ready");
    sleep(Duration::from_millis(100)).await;
    Ok(stack)
}

async fn flash_and_monitor(
    elf_path: PathBuf,
    no_monitor: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    info!("Flashing firmware: {}", elf_path.display());

    // Read and convert ELF to binary
    let elf_data = std::fs::read(&elf_path)?;
    let firmware = nrf_dfu::elf_to_bin(&elf_data)?;
    info!("Firmware size: {} bytes", firmware.len());

    // Try to connect to running device and send reboot-to-dfu
    let need_wait_for_dfu = if let Some(port) = nrf_dfu::find_adafruit_serial_port() {
        info!("Device already in DFU mode at {}", port);
        false
    } else {
        // Try to connect via ergot and reboot to DFU
        info!("Attempting to reboot device to DFU mode...");
        match try_reboot_to_dfu().await {
            Ok(()) => {
                info!("Reboot command sent, waiting for DFU device...");
                true
            }
            Err(e) => {
                warn!("Could not connect to device via ergot: {}", e);
                warn!("Please manually put device in DFU mode (double-tap reset)");
                true
            }
        }
    };

    // Wait for DFU device to appear
    if need_wait_for_dfu {
        sleep(Duration::from_millis(500)).await;
    }

    let port = nrf_dfu::wait_for_dfu_device(Duration::from_secs(10)).await?;
    info!("Found DFU device at {}", port);

    // Small delay for port to stabilize
    sleep(Duration::from_millis(200)).await;

    // Flash firmware
    let mut dfu = nrf_dfu::DfuConnection::open(&port).await?;
    dfu.flash(&firmware).await?;

    println!("Firmware flashed successfully!");

    if no_monitor {
        return Ok(());
    }

    // Wait for device to reboot and connect for logs
    println!("Waiting for device to reboot...");
    sleep(Duration::from_secs(2)).await;

    let stack = connect_usb().await?;
    println!("Connected! Streaming logs (Ctrl+C to stop)...");
    stream_logs_usb(stack).await
}

async fn try_reboot_to_dfu() -> Result<(), Box<dyn std::error::Error>> {
    let stack: RouterStack = RouterStack::new();

    // Quick scan for device
    let device = tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let devices = find_new_devices(&HashSet::new()).await;
            if let Some(dev) = devices.into_iter().next() {
                return dev;
            }
            sleep(Duration::from_millis(200)).await;
        }
    })
    .await
    .map_err(|_| "No device found")?;

    let _hdl = register_router_interface(&stack, device, MTU, OUT_BUFFER_SIZE)
        .await
        .map_err(|e| format!("Failed to register: {:?}", e))?;

    sleep(Duration::from_millis(100)).await;

    // Send reboot command (don't wait for response)
    let _ = timeout(
        Duration::from_millis(500),
        stack
            .endpoints()
            .request::<RebootToDfuEndpoint>(Address::unknown(), &(), None),
    )
    .await;

    Ok(())
}

// USB-specific functions for flash_and_monitor
async fn stream_logs_usb(stack: RouterStack) -> Result<(), Box<dyn std::error::Error>> {
    info!("Streaming logs from device (Ctrl+C to stop)...");
    println!("--- Device Logs ---");

    let receiver = stack
        .topics()
        .heap_borrowed_topic_receiver::<ErgotFmtRxTopic>(1024, None, 256);
    let receiver = pin!(receiver);
    let mut sub = receiver.subscribe();

    loop {
        let msg = sub.recv().await;
        if let Some(Ok(access)) = msg.try_access() {
            let level = match access.t.level {
                ergot::fmtlog::Level::Error => "ERROR",
                ergot::fmtlog::Level::Warn => "WARN",
                ergot::fmtlog::Level::Info => "INFO",
                ergot::fmtlog::Level::Debug => "DEBUG",
                ergot::fmtlog::Level::Trace => "TRACE",
            };
            println!("[{}] {}", level, access.t.inner);
        }
    }
}

// Generic functions that work with either USB or BLE stack
async fn stream_logs_generic(stack: ErgotStack) -> Result<(), Box<dyn std::error::Error>> {
    info!("Streaming logs from device (Ctrl+C to stop)...");
    println!("--- Device Logs ---");

    match stack {
        ErgotStack::Usb(stack) => {
            let receiver = stack
                .topics()
                .heap_borrowed_topic_receiver::<ErgotFmtRxTopic>(1024, None, 256);
            let receiver = pin!(receiver);
            let mut sub = receiver.subscribe();

            loop {
                let msg = sub.recv().await;
                if let Some(Ok(access)) = msg.try_access() {
                    print_log_message(&access.t);
                }
            }
        }
        ErgotStack::Ble(stack) => {
            let receiver = stack
                .topics()
                .heap_borrowed_topic_receiver::<ErgotFmtRxTopic>(1024, None, 256);
            let receiver = pin!(receiver);
            let mut sub = receiver.subscribe();

            loop {
                let msg = sub.recv().await;
                if let Some(Ok(access)) = msg.try_access() {
                    print_log_message(&access.t);
                }
            }
        }
    }
}

fn print_log_message(msg: &ergot::fmtlog::ErgotFmtRx<'_>) {
    let level = match msg.level {
        ergot::fmtlog::Level::Error => "ERROR",
        ergot::fmtlog::Level::Warn => "WARN",
        ergot::fmtlog::Level::Info => "INFO",
        ergot::fmtlog::Level::Debug => "DEBUG",
        ergot::fmtlog::Level::Trace => "TRACE",
    };
    println!("[{}] {}", level, msg.inner);
}

async fn set_time_generic(stack: ErgotStack) -> Result<(), Box<dyn std::error::Error>> {
    let now = chrono::Utc::now().timestamp() as u64;
    info!("Setting device time to {} ({})", now, chrono::Utc::now());

    let result = match stack {
        ErgotStack::Usb(stack) => {
            timeout(
                Duration::from_secs(2),
                stack
                    .endpoints()
                    .request::<SetTimeEndpoint>(Address::unknown(), &now, None),
            )
            .await
        }
        ErgotStack::Ble(stack) => {
            timeout(
                Duration::from_secs(2),
                stack
                    .endpoints()
                    .request::<SetTimeEndpoint>(Address::unknown(), &now, None),
            )
            .await
        }
    };

    match result {
        Ok(Ok(_)) => {
            println!("Time set successfully to {}", chrono::Utc::now());
            Ok(())
        }
        Ok(Err(e)) => {
            error!("Failed to set time: {:?}", e);
            Err("Failed to set time".into())
        }
        Err(_) => {
            error!("Timeout waiting for response");
            Err("Timeout".into())
        }
    }
}

async fn get_time_generic(stack: ErgotStack) -> Result<(), Box<dyn std::error::Error>> {
    info!("Getting device time...");

    let result = match stack {
        ErgotStack::Usb(stack) => {
            timeout(
                Duration::from_secs(2),
                stack
                    .endpoints()
                    .request::<GetTimeEndpoint>(Address::unknown(), &(), None),
            )
            .await
        }
        ErgotStack::Ble(stack) => {
            timeout(
                Duration::from_secs(2),
                stack
                    .endpoints()
                    .request::<GetTimeEndpoint>(Address::unknown(), &(), None),
            )
            .await
        }
    };

    match result {
        Ok(Ok(ts)) => {
            if let Some(dt) = chrono::DateTime::from_timestamp(ts as i64, 0) {
                println!("Device time: {} ({})", ts, dt);
            } else {
                println!("Device time: {} (invalid timestamp)", ts);
            }
            Ok(())
        }
        Ok(Err(e)) => {
            error!("Failed to get time: {:?}", e);
            Err("Failed to get time".into())
        }
        Err(_) => {
            error!("Timeout waiting for response");
            Err("Timeout".into())
        }
    }
}

async fn reboot_dfu_generic(stack: ErgotStack) -> Result<(), Box<dyn std::error::Error>> {
    info!("Sending reboot to DFU command...");

    // Don't wait for response - device will reboot immediately
    let result = match stack {
        ErgotStack::Usb(stack) => {
            timeout(
                Duration::from_millis(500),
                stack
                    .endpoints()
                    .request::<RebootToDfuEndpoint>(Address::unknown(), &(), None),
            )
            .await
        }
        ErgotStack::Ble(stack) => {
            timeout(
                Duration::from_millis(500),
                stack
                    .endpoints()
                    .request::<RebootToDfuEndpoint>(Address::unknown(), &(), None),
            )
            .await
        }
    };

    // We expect a timeout since the device reboots
    match result {
        Ok(Ok(_)) => {
            println!("Device acknowledged reboot command");
        }
        Ok(Err(_)) | Err(_) => {
            println!("Reboot command sent (device may have already rebooted)");
        }
    }

    println!("Device should now be in bootloader mode.");
    println!("You can use UF2 drag-and-drop to update firmware.");
    Ok(())
}
