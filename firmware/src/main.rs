//! Voyager firmware — nRF52840 (nice!nano v2).
//!
//! This is the top-level entry point.  It initialises hardware peripherals,
//! spawns background tasks, and wires together the BLE, USB, and display
//! subsystems.  Domain logic lives in dedicated modules:
//!
//! - [`resources`] — pin/peripheral assignments (single source of truth)
//! - [`ble`]       — BLE advertising, GATT server, SoftDevice Controller
//! - [`display`]   — OLED rendering loop
//! - [`leds`]      — LED blink task
//! - [`battery`]   — battery voltage measurement
//! - [`usb_logger`] — CDC-ACM log transport

#![no_std]
#![no_main]

mod battery;
mod ble;
mod display;
mod leds;
mod resources;
mod usb_logger;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals::RNG;
use embassy_nrf::saadc::{self, Saadc, VddhDiv5Input};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::usb::Driver;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{Peri, bind_interrupts, pac, peripherals, rng, usb};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use log::info;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use panic_halt as _;
use portable_atomic::{AtomicU32, Ordering};
use resources::*;
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};
use static_cell::StaticCell;

// ---------------------------------------------------------------------------
// Shared state — accessed from multiple modules via `crate::XXX`
// ---------------------------------------------------------------------------

/// USB CDC-ACM logger (1 KiB ring buffer).
static LOGGER: usb_logger::UsbLogger<1024> = usb_logger::UsbLogger::new();

/// Parse a decimal ASCII string to `u32` at compile time.
/// Used to convert the `BUILD_TIMESTAMP` env var set by `build.rs`.
const fn parse_u32(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut result: u32 = 0;
    let mut i = 0;
    while i < bytes.len() {
        let digit = bytes[i] - b'0';
        result = result * 10 + digit as u32;
        i += 1;
    }
    result
}

/// Unix timestamp captured at compile time (from `build.rs`).
/// Used as the initial value for [`CURRENT_TIME`].
pub(crate) const BUILD_TIMESTAMP: u32 = parse_u32(env!("BUILD_TIMESTAMP"));

/// Current Unix time in seconds.  Incremented every second by
/// [`time_tick_task`] and can be overwritten via BLE (see [`ble`]).
pub(crate) static CURRENT_TIME: AtomicU32 = AtomicU32::new(BUILD_TIMESTAMP);

/// Last measured battery voltage in millivolts.
/// Updated every 60 s by [`battery::battery_task`].
pub(crate) static BATTERY_MV: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Interrupt bindings — maps hardware IRQs to embassy/nrf-sdc handlers.
// Kept in main.rs because `Irqs` is referenced by driver constructors below.
// ---------------------------------------------------------------------------

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler, nrf_sdc::mpsl::ClockInterruptHandler;
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    SAADC => saadc::InterruptHandler;
});

// ---------------------------------------------------------------------------
// Small tasks that live in main (too tiny for their own module)
// ---------------------------------------------------------------------------

/// Increments [`CURRENT_TIME`] by one every second.
#[embassy_executor::task]
async fn time_tick_task() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        CURRENT_TIME.fetch_add(1, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialise all nRF52840 peripherals with default config.
    let p = embassy_nrf::init(Default::default());

    // Split peripherals into named groups (see resources.rs for the mapping).
    let r = split_resources!(p);

    // P0.13 controls the 3.3 V regulator on the nice!nano v2 board.
    // Driving it HIGH keeps VCC enabled; dropping it would cut power.
    let _vcc_enable = Output::new(r.power.vcc_enable, Level::High, OutputDrive::Standard);

    // --- Spawn background tasks ---

    spawner.spawn(leds::led_task(r.leds)).unwrap();
    spawner.spawn(time_tick_task()).unwrap();

    // Set up the SAADC for battery measurement (VDDH/5 internal channel,
    // 16× hardware oversampling for a cleaner reading).
    let ch_vddh = saadc::ChannelConfig::single_ended(VddhDiv5Input);
    let mut saadc_config = saadc::Config::default();
    saadc_config.oversample = saadc::Oversample::OVER16X;
    let saadc_inst = Saadc::new(r.battery.saadc, Irqs, saadc_config, [ch_vddh]);
    spawner.spawn(battery::battery_task(saadc_inst)).unwrap();

    // Initialise the USB CDC-ACM logger so `log::info!` etc. are available.
    LOGGER.init(log::LevelFilter::Info);

    // The USB peripheral needs the high-frequency clock running.
    pac::CLOCK.tasks_hfclkstart().write_value(1);
    while pac::CLOCK.events_hfclkstarted().read() != 1 {}
    info!("[init] HF clock started");

    // --- BLE: MPSL + SoftDevice Controller ---

    // MPSL manages radio timeslots and clocking for the BLE stack.
    let mpsl_p = mpsl::Peripherals::new(
        r.mpsl.rtc0,
        r.mpsl.timer0,
        r.mpsl.temp,
        r.mpsl.ppi_ch19,
        r.mpsl.ppi_ch30,
        r.mpsl.ppi_ch31,
    );
    // Use the internal RC oscillator as the low-frequency clock source.
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(ble::mpsl_task(&*mpsl)).unwrap();

    // Build the SoftDevice Controller (BLE link-layer).
    let sdc_p = sdc::Peripherals::new(
        r.sdc.ppi_ch17,
        r.sdc.ppi_ch18,
        r.sdc.ppi_ch20,
        r.sdc.ppi_ch21,
        r.sdc.ppi_ch22,
        r.sdc.ppi_ch23,
        r.sdc.ppi_ch24,
        r.sdc.ppi_ch25,
        r.sdc.ppi_ch26,
        r.sdc.ppi_ch27,
        r.sdc.ppi_ch28,
        r.sdc.ppi_ch29,
    );
    // ManuallyDrop: the RNG driver is borrowed by the SDC for its lifetime,
    // so we must not drop it while the SDC is alive.
    let mut rng_driver = core::mem::ManuallyDrop::new(rng::Rng::new(r.ble.rng, Irqs));
    let mut sdc_mem = sdc::Mem::<4720>::new();
    let sdc = ble::build_sdc(sdc_p, &mut rng_driver, mpsl, &mut sdc_mem).unwrap();
    info!("[init] BLE stack ready");

    // --- I2C display ---

    // Short delay to let the SSD1306 finish its power-on reset.
    Timer::after_millis(100).await;

    static TWIM_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    let i2c = Twim::new(
        r.display.twim,
        Irqs,
        r.display.sda,
        r.display.scl,
        twim_config,
        TWIM_BUF.init([0; 2048]),
    );
    let display = Ssd1306Async::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    info!("[init] I2C + display configured");

    // Hand off to the main run loop.
    run(sdc, r.usb.usbd, display).await;
}

// ---------------------------------------------------------------------------
// Main run loop — runs BLE, USB, and display concurrently
// ---------------------------------------------------------------------------

/// Top-level async loop that drives three subsystems in parallel:
///
/// 1. **BLE** — advertising + GATT connection handling
/// 2. **USB** — CDC-ACM device, log output, and bootloader trigger
/// 3. **Display** — OLED rendering
///
/// None of these futures ever return, so this function runs forever.
async fn run(
    sdc: nrf_sdc::SoftdeviceController<'_>,
    usbd: Peri<'_, peripherals::USBD>,
    display: display::Display<'_>,
) {
    // --- USB CDC-ACM setup ---

    let driver = Driver::new(usbd, Irqs, HardwareVbusDetect::new(Irqs));

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Voyager");
    config.product = Some("Voyager CDC");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let (mut sender, receiver, control) = class.split_with_control();
    let mut usb = builder.build();

    // --- Futures ---

    // Drives the USB device stack (handles enumeration, transfers, etc.).
    let usb_fut = usb.run();

    // Reads log messages from the ring buffer and sends them over CDC-ACM.
    // Waits for a USB host to connect before starting.
    let log_fut = async {
        let mut buf = [0u8; 64];
        sender.wait_connection().await;
        loop {
            let n = LOGGER.read(&mut buf).await;
            if let Err(EndpointError::Disabled) = sender.write_packet(&buf[..n]).await {
                sender.wait_connection().await;
            }
            // USB CDC requires a zero-length packet after a full-size packet
            // to signal end-of-transfer.
            if n == 64
                && let Err(EndpointError::Disabled) = sender.write_packet(&[]).await
            {
                sender.wait_connection().await;
            }
        }
    };

    // Monitors the CDC-ACM control line.  When the host sets the baud rate
    // to 1200 bps it triggers a system reset into the UF2 bootloader
    // (standard "double-tap" convention used by Arduino/nice!nano boards).
    let control_fut = async {
        loop {
            control.control_changed().await;
            if receiver.line_coding().data_rate() == 1200 {
                Timer::after(Duration::from_millis(100)).await;
                pac::POWER.gpregret().write(|w| w.set_gpregret(0x57));
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    };

    // --- Run everything concurrently ---

    let ble_fut = ble::run(sdc);
    let display_fut = display::run(display);
    let usb_group = async {
        embassy_futures::join::join3(usb_fut, log_fut, control_fut).await;
    };

    join(join(ble_fut, usb_group), display_fut).await;
}
