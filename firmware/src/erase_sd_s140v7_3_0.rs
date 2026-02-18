//! Erases the SoftDevice region (0x1000-0x27000) for boards with S140 v7.3.0
//! (e.g. Seeed XIAO nRF52840 Sense with bootloader v0.6.1).
//! After erasing, resets into bootloader via GPREGRET.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use {embassy_nrf as _, nrf_mpsl as _, panic_halt as _};

#[entry]
fn main() -> ! {
    const NVMC_BASE: u32 = 0x4001_E000;
    const NVMC_READY: *const u32 = (NVMC_BASE + 0x400) as *const u32;
    const NVMC_CONFIG: *mut u32 = (NVMC_BASE + 0x504) as *mut u32;
    const NVMC_ERASEPAGE: *mut u32 = (NVMC_BASE + 0x508) as *mut u32;

    unsafe { NVMC_CONFIG.write_volatile(2) };
    while unsafe { NVMC_READY.read_volatile() } == 0 {}

    // Erase pages from 0x1000 to 0x27000 (S140 v7.3.0 region)
    let mut addr: u32 = 0x1000;
    while addr < 0x27000 {
        unsafe { NVMC_ERASEPAGE.write_volatile(addr) };
        while unsafe { NVMC_READY.read_volatile() } == 0 {}
        addr += 4096;
    }

    unsafe { NVMC_CONFIG.write_volatile(0) };
    while unsafe { NVMC_READY.read_volatile() } == 0 {}

    const POWER_GPREGRET: *mut u32 = (0x4000_0000 + 0x51C) as *mut u32;
    unsafe { POWER_GPREGRET.write_volatile(0x57) };
    cortex_m::peripheral::SCB::sys_reset();
}
