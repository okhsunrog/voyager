use core::fmt::Write as _;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use log::{Metadata, Record};

type CS = CriticalSectionRawMutex;

pub struct UsbLogger<const N: usize> {
    buffer: Pipe<CS, N>,
}

impl<const N: usize> UsbLogger<N> {
    pub const fn new() -> Self {
        Self {
            buffer: Pipe::new(),
        }
    }

    pub fn init(&'static self, level: log::LevelFilter) {
        unsafe {
            let _ = log::set_logger_racy(self).map(|()| log::set_max_level_racy(level));
        }
    }

    pub async fn read(&self, buf: &mut [u8]) -> usize {
        self.buffer.read(buf).await
    }
}

impl<const N: usize> log::Log for UsbLogger<N> {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let _ = write!(Writer(&self.buffer), "{}\r\n", record.args());
        }
    }

    fn flush(&self) {}
}

struct Writer<'d, const N: usize>(&'d Pipe<CS, N>);

impl<const N: usize> core::fmt::Write for Writer<'_, N> {
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        let b = s.as_bytes();
        if let Ok(n) = self.0.try_write(b) {
            if n < b.len() {
                let _ = self.0.try_write(&b[n..]);
            }
        }
        Ok(())
    }
}
