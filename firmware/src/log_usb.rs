//! Log sink that sends logs over ergot USB
//!
//! Uses ergot's built-in fmt logging which is synchronous and non-blocking.

use ergot::fmt;
use log::{LevelFilter, Metadata, Record};

use crate::usb_ergot::USB_STACK;

/// The global logger instance
struct ErgotLogger;

impl log::Log for ErgotLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        match record.level() {
            log::Level::Error => USB_STACK.error_fmt(fmt!("[{}] {}", record.target(), record.args())),
            log::Level::Warn => USB_STACK.warn_fmt(fmt!("[{}] {}", record.target(), record.args())),
            log::Level::Info => USB_STACK.info_fmt(fmt!("[{}] {}", record.target(), record.args())),
            log::Level::Debug => USB_STACK.debug_fmt(fmt!("[{}] {}", record.target(), record.args())),
            log::Level::Trace => USB_STACK.trace_fmt(fmt!("[{}] {}", record.target(), record.args())),
        }
    }

    fn flush(&self) {}
}

static LOGGER: ErgotLogger = ErgotLogger;

/// Initialize the ergot USB logger.
/// Must be called after USB_STACK is initialized.
pub fn init() {
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(LevelFilter::Info))
        .ok();
}
