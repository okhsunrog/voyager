#![no_std]
#![no_main]

mod fmt;
#[cfg(feature = "log-usb")]
mod log_usb;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(all(feature = "log-rtt", not(feature = "defmt")))]
use rtt_target::rtt_init_log;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "ble")]
use nrf_mpsl as _;

use voyager_display::{
    Bmp, Distance, advance_scroll, render_info, render_scrolling_bmp,
};
#[cfg(any(feature = "usb", feature = "ble"))]
use core::pin::pin;
use embassy_executor::Spawner;
#[cfg(any(feature = "usb", feature = "ble"))]
use embassy_futures::select::select;
#[cfg(feature = "ble")]
use embassy_futures::select::{Either3, select3};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
#[cfg(feature = "ble")]
use embassy_nrf::mode::Async;
#[cfg(feature = "usb")]
use embassy_nrf::pac::FICR;
#[cfg(any(feature = "usb", feature = "ble"))]
use embassy_nrf::peripherals::RNG;
#[cfg(feature = "usb")]
use embassy_nrf::peripherals::USBD;
#[cfg(any(feature = "usb", feature = "ble"))]
use embassy_nrf::rng;
use embassy_nrf::twim::{self, Twim};
#[cfg(feature = "usb")]
use embassy_nrf::usb;
#[cfg(feature = "usb")]
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{bind_interrupts, peripherals};
#[cfg(feature = "ble")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "ble")]
use embassy_sync::channel::Channel;
use embassy_time::Timer;
#[cfg(feature = "usb")]
use embassy_usb::driver::Driver;
#[cfg(feature = "usb")]
use embassy_usb::{Config, UsbDevice};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
#[cfg(feature = "usb")]
use ergot::exports::bbq2::prod_cons::framed::FramedConsumer;
#[cfg(any(feature = "usb", feature = "ble"))]
use ergot::exports::bbq2::traits::coordination::cas::AtomicCoord;
#[cfg(feature = "usb")]
use ergot::toolkits::embassy_usb_v0_5 as usb_kit;
#[cfg(feature = "ble")]
use ergot::toolkits::embedded_io_async_v0_6 as eio_kit;
use fmt::info;
#[cfg(any(feature = "usb", feature = "ble"))]
use voyager_icd::{GetTimeEndpoint, RebootToDfuEndpoint, SetTimeEndpoint};
#[cfg(any(feature = "usb", feature = "ble"))]
use mutex::raw_impls::single_core_thread_mode::ThreadModeRawMutex;
#[cfg(feature = "ble")]
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
#[cfg(feature = "ble")]
use nrf_sdc::{self as sdc, mpsl};
use portable_atomic::{AtomicU32, Ordering};
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};
use static_cell::{ConstStaticCell, StaticCell};
#[cfg(feature = "ble")]
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

// ============================================================================
// Shared Endpoint Handlers
// ============================================================================

#[cfg(any(feature = "usb", feature = "ble"))]
async fn handle_set_time(ts: u32) {
    CURRENT_TIME.store(ts, Ordering::Relaxed);
    info!("Time set to: {}", ts);
}

#[cfg(any(feature = "usb", feature = "ble"))]
fn handle_get_time() -> u32 {
    CURRENT_TIME.load(Ordering::Relaxed)
}

#[cfg(any(feature = "usb", feature = "ble"))]
async fn handle_reboot_to_dfu() -> ! {
    info!("Rebooting to bootloader...");
    Timer::after_millis(100).await;
    reboot_to_bootloader()
}

// Interrupt bindings - conditionally include based on features
bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    #[cfg(any(feature = "usb", feature = "ble"))]
    RNG => rng::InterruptHandler<RNG>;
    #[cfg(feature = "ble")]
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    #[cfg(all(feature = "ble", feature = "usb"))]
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler, usb::vbus_detect::InterruptHandler;
    #[cfg(all(feature = "ble", not(feature = "usb")))]
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    #[cfg(all(feature = "usb", not(feature = "ble")))]
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    #[cfg(feature = "ble")]
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    #[cfg(feature = "ble")]
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    #[cfg(feature = "ble")]
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    #[cfg(feature = "usb")]
    USBD => usb::InterruptHandler<USBD>;
});

// Store timestamp as seconds since Unix epoch
static CURRENT_TIME: AtomicU32 = AtomicU32::new(0);

type Display = Ssd1306Async<
    I2CInterface<Twim<'static>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsModeAsync<DisplaySize128x64>,
>;

// ============================================================================
// USB Ergot Setup
// ============================================================================

#[cfg(feature = "usb")]
mod usb_ergot {
    use super::*;

    pub const USB_OUT_QUEUE_SIZE: usize = 4096;
    pub const USB_MAX_PACKET_SIZE: usize = 1024;

    pub type AppDriver = usb::Driver<'static, HardwareVbusDetect>;
    pub type UsbRxWorker = usb_kit::RxWorker<&'static UsbQueue, ThreadModeRawMutex, AppDriver>;
    pub type UsbStack = usb_kit::Stack<&'static UsbQueue, ThreadModeRawMutex>;
    pub type UsbQueue = usb_kit::Queue<USB_OUT_QUEUE_SIZE, AtomicCoord>;

    pub static USB_STACK: UsbStack =
        usb_kit::new_target_stack(USB_OUTQ.framed_producer(), USB_MAX_PACKET_SIZE as u16);
    pub static USB_STORAGE: usb_kit::WireStorage<256, 256, 64, 256> = usb_kit::WireStorage::new();
    pub static USB_OUTQ: UsbQueue = usb_kit::Queue::new();

    pub fn usb_config(serial: &'static str) -> Config<'static> {
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

    pub fn get_unique_id() -> u64 {
        let lower = FICR.deviceid(0).read() as u64;
        let upper = FICR.deviceid(1).read() as u64;
        (upper << 32) | lower
    }

    #[embassy_executor::task]
    pub async fn usb_task(mut usb: UsbDevice<'static, AppDriver>) {
        usb.run().await;
    }

    #[embassy_executor::task]
    pub async fn usb_run_rx(rcvr: UsbRxWorker, recv_buf: &'static mut [u8]) {
        rcvr.run(recv_buf, usb_kit::USB_FS_MAX_PACKET_SIZE).await;
    }

    #[embassy_executor::task]
    pub async fn usb_run_tx(
        mut ep_in: <AppDriver as Driver<'static>>::EndpointIn,
        rx: FramedConsumer<&'static UsbQueue>,
    ) {
        usb_kit::tx_worker::<AppDriver, USB_OUT_QUEUE_SIZE, AtomicCoord>(
            &mut ep_in,
            rx,
            usb_kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
            usb_kit::USB_FS_MAX_PACKET_SIZE,
        )
        .await;
    }

    #[embassy_executor::task]
    pub async fn usb_pingserver() {
        USB_STACK.services().ping_handler::<4>().await;
    }

    #[embassy_executor::task]
    pub async fn usb_time_server() {
        let set_socket = USB_STACK
            .endpoints()
            .bounded_server::<SetTimeEndpoint, 4>(Some("time"));
        let set_socket = pin!(set_socket);
        let mut set_hdl = set_socket.attach();

        let get_socket = USB_STACK
            .endpoints()
            .bounded_server::<GetTimeEndpoint, 4>(Some("time"));
        let get_socket = pin!(get_socket);
        let mut get_hdl = get_socket.attach();

        loop {
            let set_fut = set_hdl.serve(async |ts| handle_set_time(*ts).await);
            let get_fut = get_hdl.serve(async |_| handle_get_time());
            select(set_fut, get_fut).await;
        }
    }

    #[embassy_executor::task]
    pub async fn usb_dfu_server() {
        let socket = USB_STACK
            .endpoints()
            .bounded_server::<RebootToDfuEndpoint, 4>(Some("dfu"));
        let socket = pin!(socket);
        let mut hdl = socket.attach();

        loop {
            let _ = hdl.serve(async |_| handle_reboot_to_dfu().await).await;
        }
    }
}

#[cfg(feature = "usb")]
use usb_ergot::*;

// ============================================================================
// BLE NUS (Nordic UART Service) with Ergot
// ============================================================================

#[cfg(feature = "ble")]
mod ble_ergot {
    use super::*;

    // NUS UUIDs
    // Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    // RX Char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (write from central)
    // TX Char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (notify to central)

    #[gatt_server]
    pub struct Server {
        pub nus: NusService,
    }

    #[gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
    pub struct NusService {
        #[characteristic(
            uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
            write,
            write_without_response
        )]
        pub rx: heapless::Vec<u8, 244>,

        #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", notify)]
        pub tx: heapless::Vec<u8, 244>,
    }

    // Channels for NUS data
    pub static NUS_RX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 244>, 8> =
        Channel::new();
    pub static NUS_TX_CHANNEL: Channel<CriticalSectionRawMutex, heapless::Vec<u8, 244>, 8> =
        Channel::new();

    // BLE Ergot Stack
    pub const BLE_OUT_QUEUE_SIZE: usize = 2048;
    pub const BLE_MAX_PACKET_SIZE: usize = 244;

    pub type BleQueue = eio_kit::Queue<BLE_OUT_QUEUE_SIZE, AtomicCoord>;
    pub type BleStack = eio_kit::Stack<&'static BleQueue, ThreadModeRawMutex>;

    pub static BLE_OUTQ: BleQueue = eio_kit::Queue::new();
    pub static BLE_STACK: BleStack =
        eio_kit::new_target_stack(BLE_OUTQ.stream_producer(), BLE_MAX_PACKET_SIZE as u16);

    /// NUS Reader - implements embedded_io_async Read
    pub struct NusReader {
        buf: heapless::Vec<u8, 244>,
        pos: usize,
    }

    impl NusReader {
        pub const fn new() -> Self {
            Self {
                buf: heapless::Vec::new(),
                pos: 0,
            }
        }
    }

    impl embedded_io_async_0_6::ErrorType for NusReader {
        type Error = core::convert::Infallible;
    }

    impl embedded_io_async_0_6::Read for NusReader {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.pos < self.buf.len() {
                let remaining = &self.buf[self.pos..];
                let to_copy = remaining.len().min(buf.len());
                buf[..to_copy].copy_from_slice(&remaining[..to_copy]);
                self.pos += to_copy;
                return Ok(to_copy);
            }

            let data = NUS_RX_CHANNEL.receive().await;
            info!("[ble ergot] RX {} bytes from NUS", data.len());

            let to_copy = data.len().min(buf.len());
            buf[..to_copy].copy_from_slice(&data[..to_copy]);

            if to_copy < data.len() {
                self.buf.clear();
                let _ = self.buf.extend_from_slice(&data[to_copy..]);
                self.pos = 0;
            }

            Ok(to_copy)
        }
    }

    /// NUS Writer - implements embedded_io_async Write
    pub struct NusWriter;

    impl embedded_io_async_0_6::ErrorType for NusWriter {
        type Error = core::convert::Infallible;
    }

    impl embedded_io_async_0_6::Write for NusWriter {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let mut vec = heapless::Vec::new();
            let to_send = buf.len().min(244);
            let _ = vec.extend_from_slice(&buf[..to_send]);
            NUS_TX_CHANNEL.send(vec).await;
            info!("[ble ergot] TX {} bytes to NUS", to_send);
            Ok(to_send)
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[embassy_executor::task]
    pub async fn ble_pingserver() {
        BLE_STACK.services().ping_handler::<4>().await;
    }

    #[embassy_executor::task]
    pub async fn ble_time_server() {
        let set_socket = BLE_STACK
            .endpoints()
            .bounded_server::<SetTimeEndpoint, 4>(Some("time"));
        let set_socket = pin!(set_socket);
        let mut set_hdl = set_socket.attach();

        let get_socket = BLE_STACK
            .endpoints()
            .bounded_server::<GetTimeEndpoint, 4>(Some("time"));
        let get_socket = pin!(get_socket);
        let mut get_hdl = get_socket.attach();

        loop {
            let set_fut = set_hdl.serve(async |ts| handle_set_time(*ts).await);
            let get_fut = get_hdl.serve(async |_| handle_get_time());
            select(set_fut, get_fut).await;
        }
    }

    #[embassy_executor::task]
    pub async fn ble_dfu_server() {
        let socket = BLE_STACK
            .endpoints()
            .bounded_server::<RebootToDfuEndpoint, 4>(Some("dfu"));
        let socket = pin!(socket);
        let mut hdl = socket.attach();

        loop {
            let _ = hdl.serve(async |_| handle_reboot_to_dfu().await).await;
        }
    }

    #[embassy_executor::task]
    pub async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
        mpsl.run().await
    }

    pub const L2CAP_TXQ: u8 = 3;
    pub const L2CAP_RXQ: u8 = 3;
    pub const CONNECTIONS_MAX: usize = 1;
    pub const L2CAP_CHANNELS_MAX: usize = 2;

    pub fn build_sdc<'d, const N: usize>(
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

    pub async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
        loop {
            if let Err(e) = runner.run().await {
                panic!("[ble_task] error: {:?}", e);
            }
        }
    }

    pub async fn gatt_events_task<P: PacketPool>(
        server: &Server<'_>,
        conn: &GattConnection<'_, '_, P>,
    ) -> Result<(), Error> {
        let rx_char = &server.nus.rx;
        let tx_char = &server.nus.tx;

        loop {
            let gatt_fut = conn.next();
            let tx_fut = NUS_TX_CHANNEL.receive();

            match select(gatt_fut, tx_fut).await {
                embassy_futures::select::Either::First(event) => match event {
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
                                let mut vec = heapless::Vec::new();
                                let _ = vec.extend_from_slice(data);
                                let _ = NUS_RX_CHANNEL.try_send(vec);
                            }
                            match event.accept() {
                                Ok(reply) => reply.send().await,
                                Err(e) => info!("[gatt] error sending response: {:?}", e),
                            }
                        }
                        GattEvent::Read(event) => match event.accept() {
                            Ok(reply) => reply.send().await,
                            Err(e) => info!("[gatt] error sending response: {:?}", e),
                        },
                        _ => {}
                    },
                    _ => {}
                },
                embassy_futures::select::Either::Second(tx_data) => {
                    info!("[nus] TX {} bytes via notify", tx_data.len());
                    if let Err(e) = tx_char.notify(conn, &tx_data).await {
                        info!("[nus] notify error: {:?}", e);
                    }
                }
            }
        }
        Ok(())
    }

    pub async fn advertise<'values, 'server, C: Controller>(
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

    pub async fn run_ble(spawner: Spawner, sdc: nrf_sdc::SoftdeviceController<'static>) {
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

        spawner.spawn(ble_pingserver().unwrap());
        spawner.spawn(ble_time_server().unwrap());
        spawner.spawn(ble_dfu_server().unwrap());

        info!("BLE stack ready (NUS + ergot), starting advertising");

        let _ = embassy_futures::join::join(ble_task(runner), async {
            loop {
                match advertise(&mut peripheral, &server).await {
                    Ok(conn) => {
                        info!("Client connected!");

                        let mut nus_reader = NusReader::new();
                        let mut nus_writer = NusWriter;

                        static FRAME_BUF: ConstStaticCell<[u8; 512]> =
                            ConstStaticCell::new([0u8; 512]);
                        static SCRATCH_BUF: ConstStaticCell<[u8; 512]> =
                            ConstStaticCell::new([0u8; 512]);

                        let frame_buf = FRAME_BUF.take();
                        let scratch_buf = SCRATCH_BUF.take();

                        let mut rx_worker =
                            eio_kit::RxWorker::new_target(&BLE_STACK, &mut nus_reader, ());

                        let gatt = gatt_events_task(&server, &conn);
                        let ergot_rx = rx_worker.run(frame_buf, scratch_buf);
                        let ergot_tx =
                            eio_kit::tx_worker(&mut nus_writer, BLE_OUTQ.stream_consumer());
                        let time_tick = async {
                            loop {
                                Timer::after_secs(1).await;
                                CURRENT_TIME.fetch_add(1, Ordering::Relaxed);
                            }
                        };

                        match select3(gatt, select(ergot_rx, ergot_tx), time_tick).await {
                            Either3::First(_) => info!("GATT task ended"),
                            Either3::Second(_) => info!("Ergot task ended"),
                            Either3::Third(_) => info!("Time tick ended"),
                        }

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
}

#[cfg(feature = "ble")]
use ble_ergot::*;

// ============================================================================
// Display
// ============================================================================

/// Greetings bitmap (pre-rendered with greeting-renderer)
static GREETINGS_BMP: &[u8] = include_bytes!("../assets/greetings.bmp");

#[embassy_executor::task]
async fn display_task(mut display: Display) {
    // Wait a bit for system to stabilize
    Timer::after_millis(100).await;

    let bmp = Bmp::<BinaryColor>::from_slice(GREETINGS_BMP).unwrap();
    let bmp_width = bmp.bounding_box().size.width as i32;

    let mut scroll_offset = 0i32;
    const SCROLL_SPEED: i32 = 2;

    loop {
        Timer::after_millis(60).await; // ~16 FPS

        let ts = CURRENT_TIME.load(Ordering::Relaxed);

        display.clear_buffer();

        // Always calculate from timestamp (0 = epoch time)
        let distance = Distance::from_unix_timestamp(if ts == 0 {
            voyager_display::EPOCH_UNIX_SECS
        } else {
            ts
        });
        let delay = distance.signal_delay();
        let _ = render_info(&mut display, &distance, &delay);

        // Render scrolling greetings
        let _ = render_scrolling_bmp(&mut display, &bmp, scroll_offset);
        scroll_offset = advance_scroll(scroll_offset, SCROLL_SPEED, bmp_width);

        let _ = display.flush().await;
    }
}

// ============================================================================
// Main
// ============================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize RTT logger for log-rtt feature (must be before any logging)
    #[cfg(all(feature = "log-rtt", not(feature = "defmt")))]
    rtt_init_log!(log::LevelFilter::Info);

    let p = embassy_nrf::init(Default::default());

    // P0.13 controls VCC on nice!nano - set HIGH to enable 3.3V power
    let _vcc_enable = Output::new(p.P0_13, Level::High, OutputDrive::Standard);
    Timer::after_millis(100).await;

    // Setup USB
    #[cfg(feature = "usb")]
    {
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

        let driver = usb::Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));
        let config = usb_config(ser_str);
        let (device, tx_impl, ep_out) = USB_STORAGE.init_ergot(driver, config);

        static USB_RX_BUF: ConstStaticCell<[u8; USB_MAX_PACKET_SIZE]> =
            ConstStaticCell::new([0u8; USB_MAX_PACKET_SIZE]);
        let rxvr: UsbRxWorker = usb_kit::RxWorker::new(&USB_STACK, ep_out);

        spawner.spawn(usb_task(device).unwrap());
        spawner.spawn(usb_run_tx(tx_impl, USB_OUTQ.framed_consumer()).unwrap());
        spawner.spawn(usb_run_rx(rxvr, USB_RX_BUF.take()).unwrap());
        spawner.spawn(usb_pingserver().unwrap());
        spawner.spawn(usb_time_server().unwrap());
        spawner.spawn(usb_dfu_server().unwrap());

        // Initialize ergot USB log sink (must be after USB stack is ready)
        #[cfg(feature = "log-usb")]
        log_usb::init();

        info!("USB ergot initialized");
    }

    // Setup display
    static TWIM_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let i2c = Twim::new(
        p.TWISPI0,
        Irqs,
        p.P0_17,
        p.P0_20,
        Default::default(),
        TWIM_BUF.init([0; 2048]),
    );

    let mut display = Ssd1306Async::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().await.unwrap();

    // Setup BLE (must be before display_task to initialize critical-section)
    #[cfg(feature = "ble")]
    {
        info!("Initializing BLE...");

        let mpsl_p =
            mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
        let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
            source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
            rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
            accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
            skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
        };

        static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
        let mpsl =
            MPSL.init(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
        spawner.spawn(mpsl_task(mpsl).unwrap());

        let sdc_p = sdc::Peripherals::new(
            p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
            p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
        );

        static RNG_CELL: StaticCell<rng::Rng<'static, Async>> = StaticCell::new();
        let rng = RNG_CELL.init(rng::Rng::new(p.RNG, Irqs));
        static SDC_MEM: StaticCell<sdc::Mem<4720>> = StaticCell::new();
        let sdc_mem = SDC_MEM.init(sdc::Mem::<4720>::new());
        let sdc = build_sdc(sdc_p, rng, mpsl, sdc_mem).unwrap();

        info!("BLE initialized");

        // Spawn display task after MPSL is initialized (provides critical-section)
        spawner.spawn(display_task(display).unwrap());

        run_ble(spawner, sdc).await;
    }

    // If no BLE, spawn display and keep main alive
    #[cfg(not(feature = "ble"))]
    {
        spawner.spawn(display_task(display).unwrap());
        loop {
            Timer::after_secs(3600).await;
        }
    }
}
