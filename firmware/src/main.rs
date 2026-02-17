#![no_std]
#![no_main]

mod usb_logger;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals::RNG;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::{bind_interrupts, pac, peripherals, Peri, rng, usb};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use log::info;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use portable_atomic::{AtomicU32, Ordering};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use voyager_display::{Bmp, Distance, advance_scroll, render_info, render_scrolling_bmp};
use {panic_halt as _};

static LOGGER: usb_logger::UsbLogger<1024> = usb_logger::UsbLogger::new();

/// Parse a decimal string to u32 at compile time
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

/// Compile-time Unix timestamp from build.rs
const BUILD_TIMESTAMP: u32 = parse_u32(env!("BUILD_TIMESTAMP"));

/// Current time in Unix seconds — single source of truth
static CURRENT_TIME: AtomicU32 = AtomicU32::new(BUILD_TIMESTAMP);

/// Greetings bitmap (pre-rendered with greeting-renderer)
static GREETINGS_BMP: &[u8] = include_bytes!("../assets/greetings.bmp");

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler, nrf_sdc::mpsl::ClockInterruptHandler;
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

/// LED blink task - 200ms on, 1800ms off
#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(200).await;
        led.set_low();
        Timer::after_millis(1800).await;
    }
}

/// Time tick task - increments CURRENT_TIME every second
#[embassy_executor::task]
async fn time_tick_task() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        CURRENT_TIME.fetch_add(1, Ordering::Relaxed);
    }
}

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;
/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 3;
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<'d, embassy_nrf::mode::Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()
        .support_peripheral()
        .peripheral_count(1)?
        .buffer_cfg(
            DefaultPacketPool::MTU as u16,
            DefaultPacketPool::MTU as u16,
            L2CAP_TXQ,
            L2CAP_RXQ,
        )?
        .build(p, rng, mpsl, mem)
}

#[gatt_server]
struct Server {
    time_service: TimeService,
}

#[gatt_service(uuid = "a2d7a6e0-5e7b-4f1c-8a3d-b2c9f0e1d4a8")]
struct TimeService {
    #[characteristic(uuid = "a2d7a6e1-5e7b-4f1c-8a3d-b2c9f0e1d4a8", read, write)]
    unix_time: u32,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // P0.13 controls VCC on nice!nano - set HIGH to enable 3.3V power
    let _vcc_enable = Output::new(p.P0_13, Level::High, OutputDrive::Standard);

    // LED blink task
    let led = Output::new(p.P0_15, Level::Low, OutputDrive::Standard);
    spawner.spawn(led_task(led)).unwrap();

    // Time tick task
    spawner.spawn(time_tick_task()).unwrap();

    LOGGER.init(log::LevelFilter::Info);

    // Start HF clock (required for USB)
    pac::CLOCK.tasks_hfclkstart().write_value(1);
    while pac::CLOCK.events_hfclkstarted().read() != 1 {}

    // --- BLE setup ---
    let mpsl_p = mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(&*mpsl)).unwrap();

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng_driver = core::mem::ManuallyDrop::new(rng::Rng::new(p.RNG, Irqs));
    let mut sdc_mem = sdc::Mem::<4720>::new();
    let sdc = build_sdc(sdc_p, &mut rng_driver, mpsl, &mut sdc_mem).unwrap();

    // --- I2C + Display setup ---
    Timer::after_millis(100).await;

    static TWIM_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let mut twim_config = embassy_nrf::twim::Config::default();
    twim_config.frequency = embassy_nrf::twim::Frequency::K400;
    let i2c = Twim::new(p.TWISPI0, Irqs, p.P0_17, p.P0_20, twim_config, TWIM_BUF.init([0; 2048]));

    let display = Ssd1306Async::new(I2CDisplayInterface::new(i2c), DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    run(sdc, p.USBD, display).await;
}

async fn run(
    sdc: nrf_sdc::SoftdeviceController<'_>,
    usbd: Peri<'_, peripherals::USBD>,
    mut display: Ssd1306Async<
        ssd1306::prelude::I2CInterface<Twim<'_>>,
        DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsModeAsync<DisplaySize128x64>,
    >,
) {
    display.init().await.unwrap();

    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("BLE address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Voyager",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    info!("Voyager initialized (build timestamp: {})", BUILD_TIMESTAMP);

    // --- USB setup ---
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

    // --- Task futures ---

    let usb_fut = usb.run();

    let log_fut = async {
        let mut buf = [0u8; 64];
        sender.wait_connection().await;
        loop {
            let n = LOGGER.read(&mut buf).await;
            if let Err(EndpointError::Disabled) = sender.write_packet(&buf[..n]).await {
                sender.wait_connection().await;
            }
            if n == 64 {
                if let Err(EndpointError::Disabled) = sender.write_packet(&[]).await {
                    sender.wait_connection().await;
                }
            }
        }
    };

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

    // BLE runner
    let ble_runner_fut = async {
        loop {
            if let Err(e) = runner.run().await {
                info!("[ble] runner error: {:?}", e);
            }
        }
    };

    // BLE advertising + connection handling
    let ble_peripheral_fut = async {
        let unix_time = server.time_service.unix_time;
        loop {
            let mut adv_data = [0; 31];
            let len = AdStructure::encode_slice(
                &[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::CompleteLocalName(b"Voyager"),
                ],
                &mut adv_data[..],
            )
            .unwrap();

            info!("[ble] advertising...");
            let advertiser = match peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..len],
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(a) => a,
                Err(e) => {
                    info!("[ble] advertise error: {:?}", e);
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            };

            let conn = match advertiser.accept().await {
                Ok(c) => match c.with_attribute_server(&server) {
                    Ok(c) => c,
                    Err(e) => {
                        info!("[ble] attribute server error: {:?}", e);
                        continue;
                    }
                },
                Err(e) => {
                    info!("[ble] accept error: {:?}", e);
                    continue;
                }
            };

            info!("[ble] connected");

            loop {
                match conn.next().await {
                    GattConnectionEvent::Disconnected { reason } => {
                        info!("[ble] disconnected: {:?}", reason);
                        break;
                    }
                    GattConnectionEvent::Gatt { event } => {
                        match &event {
                            GattEvent::Write(event) => {
                                if event.handle() == unix_time.handle {
                                    let data = event.data();
                                    if data.len() == 4 {
                                        let ts = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                                        CURRENT_TIME.store(ts, Ordering::Relaxed);
                                        info!("[ble] time set to {}", ts);
                                    }
                                }
                            }
                            GattEvent::Read(event) => {
                                if event.handle() == unix_time.handle {
                                    let ts = CURRENT_TIME.load(Ordering::Relaxed);
                                    info!("[ble] time read: {}", ts);
                                }
                            }
                            _ => {}
                        }
                        match event.accept() {
                            Ok(reply) => reply.send().await,
                            Err(e) => info!("[gatt] error sending response: {:?}", e),
                        };
                    }
                    _ => {}
                }
            }
        }
    };

    // Display rendering
    let display_fut = async {
        let bmp = Bmp::<BinaryColor>::from_slice(GREETINGS_BMP).unwrap();
        let bmp_width = bmp.bounding_box().size.width as i32;
        let mut scroll_offset = 0i32;
        const SCROLL_SPEED: i32 = 3;

        loop {
            let current_time = CURRENT_TIME.load(Ordering::Relaxed);

            display.clear_buffer();

            let distance = Distance::from_unix_timestamp(current_time);
            let delay = distance.signal_delay();
            let _ = render_info(&mut display, &distance, &delay);

            let _ = render_scrolling_bmp(&mut display, &bmp, scroll_offset);
            scroll_offset = advance_scroll(scroll_offset, SCROLL_SPEED, bmp_width);

            let _ = display.flush().await;

            Timer::after_millis(50).await;
        }
    };

    // Combine BLE futures
    let ble_fut = join(ble_runner_fut, ble_peripheral_fut);

    // Combine USB futures
    let usb_group = async {
        embassy_futures::join::join3(usb_fut, log_fut, control_fut).await;
    };

    // Run everything
    let connectivity_fut = join(ble_fut, usb_group);
    join(connectivity_fut, display_fut).await;
}
