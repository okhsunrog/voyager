#![no_std]
#![no_main]

mod fmt;
mod icd;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

// Link in nrf-mpsl for critical-section implementation
use nrf_mpsl as _;

use core::pin::pin;

use chrono::{DateTime, Datelike, Timelike, Utc};
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::mode::Async;
use embassy_nrf::pac::FICR;
use embassy_nrf::peripherals::{RNG, USBD};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{bind_interrupts, peripherals, rng, usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embassy_usb::driver::Driver;
use embassy_usb::{Config, UsbDevice};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ergot::exports::bbq2::{prod_cons::framed::FramedConsumer, traits::coordination::cas::AtomicCoord};
use ergot::toolkits::embassy_usb_v0_5 as kit;
use fmt::info;
use heapless::String;
use icd::{GetTimeEndpoint, RebootToDfuEndpoint, SetTimeEndpoint};
use mutex::raw_impls::single_core_thread_mode::ThreadModeRawMutex;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use portable_atomic::{AtomicU64, Ordering};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};
use static_cell::{ConstStaticCell, StaticCell};
use trouble_host::prelude::*;

// Magic values for Adafruit bootloader (written to GPREGRET before reset)
const DFU_MAGIC_UF2_RESET: u32 = 0x57; // Enter UF2 bootloader mode (CDC + MSC)

/// Reboot into bootloader DFU mode
fn reboot_to_bootloader() -> ! {
    embassy_nrf::pac::POWER
        .gpregret()
        .write(|w| w.set_gpregret(DFU_MAGIC_UF2_RESET as u8));
    cortex_m::peripheral::SCB::sys_reset()
}

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler, usb::vbus_detect::InterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    USBD => usb::InterruptHandler<USBD>;
});

// Store timestamp as seconds since Unix epoch
static CURRENT_TIME: AtomicU64 = AtomicU64::new(0);

type Display = Ssd1306Async<
    I2CInterface<Twim<'static>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsModeAsync<DisplaySize128x64>,
>;

// ============================================================================
// USB Ergot Setup
// ============================================================================

const OUT_QUEUE_SIZE: usize = 4096;
const MAX_PACKET_SIZE: usize = 1024;

type AppDriver = usb::Driver<'static, HardwareVbusDetect>;
type RxWorker = kit::RxWorker<&'static Queue, ThreadModeRawMutex, AppDriver>;
type Stack = kit::Stack<&'static Queue, ThreadModeRawMutex>;
type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;

static STACK: Stack = kit::new_target_stack(OUTQ.framed_producer(), MAX_PACKET_SIZE as u16);
static STORAGE: kit::WireStorage<256, 256, 64, 256> = kit::WireStorage::new();
static OUTQ: Queue = kit::Queue::new();

fn usb_config(serial: &'static str) -> Config<'static> {
    let mut config = Config::new(0x16c0, 0x27DD);
    config.manufacturer = Some("Voyager");
    config.product = Some("voyager-nrf");
    config.serial_number = Some(serial);
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;
    config
}

fn get_unique_id() -> u64 {
    let lower = FICR.deviceid(0).read() as u64;
    let upper = FICR.deviceid(1).read() as u64;
    (upper << 32) | lower
}

#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn run_rx(rcvr: RxWorker, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, kit::USB_FS_MAX_PACKET_SIZE).await;
}

#[embassy_executor::task]
async fn run_tx(mut ep_in: <AppDriver as Driver<'static>>::EndpointIn, rx: FramedConsumer<&'static Queue>) {
    kit::tx_worker::<AppDriver, OUT_QUEUE_SIZE, AtomicCoord>(
        &mut ep_in,
        rx,
        kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

#[embassy_executor::task]
async fn pingserver() {
    STACK.services().ping_handler::<4>().await;
}

#[embassy_executor::task]
async fn time_server() {
    // Set time endpoint
    let set_socket = STACK
        .endpoints()
        .bounded_server::<SetTimeEndpoint, 4>(Some("time"));
    let set_socket = pin!(set_socket);
    let mut set_hdl = set_socket.attach();

    // Get time endpoint
    let get_socket = STACK
        .endpoints()
        .bounded_server::<GetTimeEndpoint, 4>(Some("time"));
    let get_socket = pin!(get_socket);
    let mut get_hdl = get_socket.attach();

    loop {
        let set_fut = set_hdl.serve(async |ts| {
            CURRENT_TIME.store(*ts, Ordering::Relaxed);
            info!("[usb] Time set to: {}", ts);
        });

        let get_fut = get_hdl.serve(async |_| CURRENT_TIME.load(Ordering::Relaxed));

        select(set_fut, get_fut).await;
    }
}

#[embassy_executor::task]
async fn dfu_server() {
    let socket = STACK
        .endpoints()
        .bounded_server::<RebootToDfuEndpoint, 4>(Some("dfu"));
    let socket = pin!(socket);
    let mut hdl = socket.attach();

    loop {
        let _ = hdl
            .serve(async |_| {
                info!("[usb] Rebooting to bootloader...");
                Timer::after_millis(100).await;
                reboot_to_bootloader();
            })
            .await;
    }
}

// ============================================================================
// BLE NUS (Nordic UART Service) Setup
// ============================================================================

// NUS UUIDs
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// RX Char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (write from central)
// TX Char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (notify to central)

#[gatt_server]
struct Server {
    nus: NusService,
}

#[gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
struct NusService {
    /// RX characteristic - central writes to this
    #[characteristic(uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e", write, write_without_response)]
    rx: heapless::Vec<u8, 244>,

    /// TX characteristic - peripheral notifies through this
    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", notify)]
    tx: heapless::Vec<u8, 244>,
}

// Channel for passing received NUS data to ergot
static NUS_RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 244>, 4> = Channel::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

const L2CAP_TXQ: u8 = 3;
const L2CAP_RXQ: u8 = 3;
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<'d, Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()?
        .support_peripheral()?
        .peripheral_count(1)?
        .buffer_cfg(
            DefaultPacketPool::MTU as u16,
            DefaultPacketPool::MTU as u16,
            L2CAP_TXQ,
            L2CAP_RXQ,
        )?
        .build(p, rng, mpsl, mem)
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let rx_char = &server.nus.rx;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => match event {
                GattEvent::Write(event) => {
                    let handle = event.handle();
                    if handle == rx_char.handle {
                        let data = event.data();
                        info!("[nus] RX {} bytes", data.len());
                        // Send to ergot via channel
                        let mut vec = heapless::Vec::new();
                        let _ = vec.extend_from_slice(data);
                        let _ = NUS_RX_CHANNEL.try_send(vec);
                    }
                    match event.accept() {
                        Ok(reply) => reply.send().await,
                        Err(e) => info!("[gatt] error sending response: {:?}", e),
                    }
                }
                GattEvent::Read(event) => {
                    match event.accept() {
                        Ok(reply) => reply.send().await,
                        Err(e) => info!("[gatt] error sending response: {:?}", e),
                    }
                }
                _ => {}
            },
            _ => {}
        }
    }
    Ok(())
}

async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"Voyager"),
        ],
        &mut adv_data[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;

    info!("[adv] advertising as 'Voyager'");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

async fn run_ble(sdc: nrf_sdc::SoftdeviceController<'static>) {
    let address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("BLE address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Voyager",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    info!("BLE stack ready (NUS service), starting advertising");

    let _ = embassy_futures::join::join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    info!("Client connected!");
                    let gatt = gatt_events_task(&server, &conn);
                    let time_tick = async {
                        loop {
                            Timer::after_secs(1).await;
                            CURRENT_TIME.fetch_add(1, Ordering::Relaxed);
                        }
                    };
                    select(gatt, time_tick).await;
                    info!("Client disconnected, resuming advertising");
                }
                Err(e) => {
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

// ============================================================================
// Display
// ============================================================================

#[embassy_executor::task]
async fn display_task(mut display: Display) {
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    loop {
        Timer::after_millis(500).await;

        let ts = CURRENT_TIME.load(Ordering::Relaxed);

        display.clear_buffer();

        if ts == 0 {
            let _ = Text::with_baseline(
                "Waiting for",
                Point::new(0, 20),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display);
            let _ = Text::with_baseline(
                "time via USB/BLE",
                Point::new(0, 35),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display);
        } else if let Some(dt) = DateTime::<Utc>::from_timestamp(ts as i64, 0) {
            let mut date_buf: String<16> = String::new();
            let mut time_buf: String<16> = String::new();
            use core::fmt::Write;
            let _ = write!(
                date_buf,
                "{:04}-{:02}-{:02}",
                dt.year(),
                dt.month(),
                dt.day()
            );
            let _ = write!(
                time_buf,
                "{:02}:{:02}:{:02}",
                dt.hour(),
                dt.minute(),
                dt.second()
            );

            let _ =
                Text::with_baseline(&date_buf, Point::new(0, 20), text_style, Baseline::Top)
                    .draw(&mut display);
            let _ =
                Text::with_baseline(&time_buf, Point::new(0, 35), text_style, Baseline::Top)
                    .draw(&mut display);
        }

        let _ = display.flush().await;
    }
}

// ============================================================================
// Main
// ============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // P0.13 controls VCC on nice!nano - set HIGH to enable 3.3V power
    let _vcc_enable = Output::new(p.P0_13, Level::High, OutputDrive::Standard);
    Timer::after_millis(100).await;

    // Get unique device ID for USB serial
    let unique_id = get_unique_id();
    static SERIAL_STRING: StaticCell<[u8; 16]> = StaticCell::new();
    let mut ser_buf = [b' '; 16];
    unique_id
        .to_be_bytes()
        .iter()
        .zip(ser_buf.chunks_exact_mut(2))
        .for_each(|(b, chs)| {
            let mut b = *b;
            for c in chs {
                *c = match b >> 4 {
                    v @ 0..10 => b'0' + v,
                    v @ 10..16 => b'A' + (v - 10),
                    _ => b'X',
                };
                b <<= 4;
            }
        });
    let ser_buf = SERIAL_STRING.init(ser_buf);
    let ser_str = core::str::from_utf8(ser_buf.as_slice()).unwrap();

    // Setup USB
    let driver = usb::Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
    let config = usb_config(ser_str);
    let (device, tx_impl, ep_out) = STORAGE.init_ergot(driver, config);

    static RX_BUF: ConstStaticCell<[u8; MAX_PACKET_SIZE]> = ConstStaticCell::new([0u8; MAX_PACKET_SIZE]);
    let rxvr: RxWorker = kit::RxWorker::new(&STACK, ep_out);

    spawner.spawn(usb_task(device).unwrap());
    spawner.spawn(run_tx(tx_impl, OUTQ.framed_consumer()).unwrap());
    spawner.spawn(run_rx(rxvr, RX_BUF.take()).unwrap());
    spawner.spawn(pingserver().unwrap());
    spawner.spawn(time_server().unwrap());
    spawner.spawn(dfu_server().unwrap());

    info!("USB ergot initialized");

    // Setup async I2C for display
    static TWIM_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let i2c = Twim::new(
        p.TWISPI0,
        Irqs,
        p.P0_17,
        p.P0_20,
        Default::default(),
        TWIM_BUF.init([0; 2048]),
    );

    // Initialize SSD1306 display
    let mut display = Ssd1306Async::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().await.unwrap();

    // Show initial message
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    Text::with_baseline("Voyager", Point::new(0, 0), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("Initializing...", Point::new(0, 15), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    display.flush().await.unwrap();

    spawner.spawn(display_task(display).unwrap());

    info!("Initializing BLE...");

    // Setup MPSL
    let mpsl_p = mpsl::Peripherals::new(
        p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31,
    );
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    // Setup SDC
    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23,
        p.PPI_CH24, p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    static RNG_CELL: StaticCell<rng::Rng<'static, Async>> = StaticCell::new();
    let rng = RNG_CELL.init(rng::Rng::new(p.RNG, Irqs));
    static SDC_MEM: StaticCell<sdc::Mem<4720>> = StaticCell::new();
    let sdc_mem = SDC_MEM.init(sdc::Mem::<4720>::new());
    let sdc = build_sdc(sdc_p, rng, mpsl, sdc_mem).unwrap();

    info!("BLE initialized");

    // Run BLE directly in main (no unsafe needed)
    run_ble(sdc).await;
}
