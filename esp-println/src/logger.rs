use super::println;

#[cfg(not(host_is_windows))]
include!(concat!(env!("OUT_DIR"), "/log_filter.rs"));

#[cfg(host_is_windows)]
include!(concat!(env!("OUT_DIR"), "\\log_filter.rs"));

/// Initialize the logger with the given maximum log level.
///
/// `ESP_LOG` environment variable will still be honored if set.
pub fn init_logger(level: log::LevelFilter) {
    unsafe {
        log::set_logger_racy(&EspLogger).unwrap();
        log::set_max_level_racy(level);
    }
}

/// Initialize the logger from the `ESP_LOG` environment variable.
pub fn init_logger_from_env() {
    unsafe {
        log::set_logger_racy(&EspLogger).unwrap();
        log::set_max_level_racy(FILTER_MAX);
    }
}

struct EspLogger;

impl log::Log for EspLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        let level = metadata.level();
        let target = metadata.target();
        is_enabled(level, target)
    }

    #[allow(unused)]
    fn log(&self, record: &log::Record) {
        if !self.enabled(&record.metadata()) {
            return;
        }

        const RESET: &str = "\u{001B}[0m";
        const RED: &str = "\u{001B}[31m";
        const GREEN: &str = "\u{001B}[32m";
        const YELLOW: &str = "\u{001B}[33m";
        const BLUE: &str = "\u{001B}[34m";
        const CYAN: &str = "\u{001B}[35m";

        #[cfg(feature = "colors")]
        let color = match record.level() {
            log::Level::Error => RED,
            log::Level::Warn => YELLOW,
            log::Level::Info => GREEN,
            log::Level::Debug => BLUE,
            log::Level::Trace => CYAN,
        };
        #[cfg(feature = "colors")]
        let reset = RESET;

        #[cfg(not(feature = "colors"))]
        let color = "";
        #[cfg(not(feature = "colors"))]
        let reset = "";

        #[cfg(feature = "timestamp")]
        println!(
            "{}{} ({}) - {}{}",
            color,
            record.level(),
            unsafe { _esp_println_timestamp() },
            record.args(),
            reset
        );
        #[cfg(not(feature = "timestamp"))]
        println!("{}{} - {}{}", color, record.level(), record.args(), reset);
    }

    fn flush(&self) {}
}

/// A user-provided hook to supply a timestamp in milliseconds for logging.
///
/// When enabled via the `"timestamp"` feature, this function should be
/// implemented to return a timestamp in milliseconds since a reference point
/// (e.g., system boot or Unix epoch).
///
/// # Example
///
/// When using [`esp-hal`], you can define this function as follows:
///
/// ```rust
/// #[no_mangle]
/// pub extern "Rust" fn _esp_println_timestamp() -> u64 {
///     esp_hal::time::Instant::now()
///         .duration_since_epoch()
///         .as_millis()
/// }
/// ```
///
/// # Notes
///
/// - If no implementations is provided, attempting to use this function will
///   result in a linker
/// error.
#[cfg(feature = "timestamp")]
extern "Rust" {
    fn _esp_println_timestamp() -> u64;
}
