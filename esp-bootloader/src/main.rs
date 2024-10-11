#![no_std]
#![no_main]

/// We arrive here after the ROM bootloader finished loading this second stage
/// bootloader from flash. The hardware is mostly uninitialized, flash cache is
/// down and the app CPU is in reset. We do have a stack, so we can do the
/// initialization in Rust.
#[esp_hal::entry] // TODO does this work for the bootloader?
fn main() -> ! {
    loop {}
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
