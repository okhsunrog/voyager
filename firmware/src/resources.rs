//! Peripheral resource assignments.
//!
//! This module uses the `assign_resources!` macro to split the nRF52840
//! peripherals into named groups. Each group is a struct that can be passed
//! to a task or init function, so every module only receives the hardware
//! it actually needs.
//!
//! Pin/peripheral mapping is defined here in one place — change a pin
//! assignment here and it propagates everywhere.

use assign_resources::assign_resources;
use embassy_nrf::{Peri, peripherals};

assign_resources! {
    // P0.13 controls the 3.3 V regulator on nice!nano v2.
    // Must be driven HIGH early in boot to keep the board powered.
    power: PowerResources {
        vcc_enable: P0_13,
    }

    // Onboard LED + three external beacon LEDs.
    leds: LedResources {
        onboard: P0_15,
        led1: P0_06,
        led2: P0_08,
        led3: P1_11,
    }

    // SAADC peripheral for battery voltage measurement via the
    // internal VDDH/5 channel (no external components needed).
    battery: BatteryResources {
        saadc: SAADC,
    }

    // Peripherals consumed by the Nordic MPSL (Multi-Protocol Service Layer),
    // which provides the radio timeslot API used by the SoftDevice Controller.
    mpsl: MpslResources {
        rtc0: RTC0,
        timer0: TIMER0,
        temp: TEMP,
        ppi_ch19: PPI_CH19,
        ppi_ch30: PPI_CH30,
        ppi_ch31: PPI_CH31,
    }

    // PPI channels consumed by the Nordic SoftDevice Controller (BLE stack).
    sdc: SdcResources {
        ppi_ch17: PPI_CH17,
        ppi_ch18: PPI_CH18,
        ppi_ch20: PPI_CH20,
        ppi_ch21: PPI_CH21,
        ppi_ch22: PPI_CH22,
        ppi_ch23: PPI_CH23,
        ppi_ch24: PPI_CH24,
        ppi_ch25: PPI_CH25,
        ppi_ch26: PPI_CH26,
        ppi_ch27: PPI_CH27,
        ppi_ch28: PPI_CH28,
        ppi_ch29: PPI_CH29,
    }

    // Hardware RNG, required by the BLE stack for address generation
    // and encryption.
    ble: BleResources {
        rng: RNG,
    }

    // I2C bus for the SSD1306 OLED display (128×64).
    // SDA = P0.17, SCL = P0.20, running at 400 kHz.
    display: DisplayResources {
        twim: TWISPI0,
        sda: P0_17,
        scl: P0_20,
    }

    // USB device peripheral for CDC-ACM serial logging.
    usb: UsbResources {
        usbd: USBD,
    }
}
