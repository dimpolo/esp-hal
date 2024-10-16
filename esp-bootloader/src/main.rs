#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![allow(
    unused,
    non_upper_case_globals,
    clippy::identity_op,
    clippy::empty_loop,
    clippy::missing_safety_doc
)]

use core::ptr::addr_of_mut;

use esp_hal::{peripherals, rtc_cntl::Swd, Config};
use esp_println::println;

#[cfg(not(esp32s3))]
compile_error!("Bootloader only implemented for ESP32-S3 for now");

/// We arrive here after the ROM bootloader finished loading this second stage
/// bootloader from flash. The hardware is mostly uninitialized, flash cache is
/// down and the app CPU is in reset. We do have a stack, so we can do the
/// initialization in Rust.
pub unsafe extern "C" fn __main_trampoline() -> ! {
    // ESP32Reset() -> Reset() -> main()
    // ESP32Reset:
    // configure_cpu_caches
    // xtensa_lx::set_stack_pointer
    // xtensa_lx_rt::zero_bss
    // stack_chk_guard.write_volatile(0xdeadbabe)
    // Reset:
    // r0::zero_bss
    // r0::init_data
    // reset_internal_timers
    // set_vecbase

    main()
}

#[export_name = "BootloaderReset"]
pub unsafe extern "C" fn main() -> ! {
    let peripherals = esp_hal::init(Config::default());
    println!("hello");

    // 1. Hardware initialization
    // bootloader_init();

    // #[cfg(CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP)]
    // If this boot is a wake up from the deep sleep then go to the short way,
    // try to load the application which worked before deep sleep.
    // It skips a lot of checks due to it was done before (while first boot).
    // bootloader_utility_load_boot_image_from_deep_sleep();

    // 2. Select the number of boot partition
    // let (bootloader_state, boot_index) = select_partition_number();

    // 3. Load the app image for booting
    // bootloader_utility_load_boot_image(bootloader_state, boot_index);
    loop {}
}

// https://github.com/espressif/esp-idf/blob/v5.3.1/components/bootloader_support/src/esp32s3/bootloader_esp32s3.c#L140
/// so far only relevant things:
/// - bootloader_clear_bss_section
/// - cache_hal_init
fn bootloader_init() -> Result<(), ()> {
    bootloader_ana_reset_config();
    bootloader_super_wdt_auto_feed();

    // protect memory region
    // FIXME bootloader_init_mem();

    // clear bss section
    bootloader_clear_bss_section();

    // init eFuse virtual mode (read eFuses to RAM)
    // FIXME esp_efuse_init_virtual_mode_in_ram();

    // config clock
    // FIXME bootloader_clock_configure(); https://github.com/espressif/esp-idf/blob/v5.3.1/components/bootloader_support/src/bootloader_clock_init.c
    // initialize console, from now on, we can use esp_log
    // TODO bootloader_console_init(); need? https://github.com/espressif/esp-idf/blob/v5.3.1/components/bootloader_support/src/bootloader_console.c
    // print 2nd bootloader banner
    // bootloader_print_banner();

    // init cache hal
    cache_hal_init();
    // init mmu
    mmu_hal_init();
    // update flash ID FIXME writes bootloader_read_flash_id() to a structure at a
    // fixed address in RAM bootloader_flash_update_id();

    // Check and run XMC startup flow
    // bootloader_flash_xmc_startup()?;
    // read bootloader header
    // bootloader_read_bootloader_header()?;
    // read chip revision and check if it's compatible to bootloader
    // bootloader_check_bootloader_validity()?;
    // initialize spi flash
    // bootloader_init_spi_flash()?;
    Ok(())
}

/// Erratum 572:
/// Disable zero-overhead loop buffer to prevent rare illegal instruction
/// exceptions while executing zero-overhead loops.
// FIXME where is this needed? not esp32s3
fn erratum_572() {
    // write special register MEMCTL
    // see https://www.cadence.com/content/dam/cadence-www/global/en_US/documents/tools/silicon-solutions/compute-ip/isa-summary.pdf
    unsafe { core::arch::asm!("wsr.memctl {0}", in(reg) 0, options(nostack)) };
}

#[cfg(esp32s3)]
fn bootloader_ana_reset_config() {
    // Enable WDT, BOD, and GLITCH reset
    bootloader_ana_super_wdt_reset_config();
    brownout_ll_ana_reset_enable();
    bootloader_ana_clock_glitch_reset_config();
}

#[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2, esp32s3))]
fn bootloader_super_wdt_auto_feed() {
    Swd::new().disable()
}

fn bootloader_clear_bss_section() {
    // These symbols come from `link.x`
    extern "C" {
        static mut _bss_start: u32;
        static mut _bss_end: u32;
    }

    unsafe { r0::zero_bss(addr_of_mut!(_bss_start), addr_of_mut!(_bss_end)) };
}

// https://github.com/espressif/esp-idf/blob/v5.3.1/components/hal/cache_hal.c
fn cache_hal_init() {
    // TODO
}

#[cfg(esp32s3)]
const CONFIG_MMU_PAGE_SIZE: MmuPageSize = MmuPageSize::MMU_PAGE_64KB;

fn mmu_hal_init() {
    // #if CONFIG_ESP_ROM_RAM_APP_NEEDS_MMU_INIT
    unsafe { esp_hal::rom_boot_cache_init() };

    // on S3: assert CONFIG_MMU_PAGE_SIZE == MMU_PAGE_64KB
    mmu_ll_set_page_size(0, CONFIG_MMU_PAGE_SIZE);

    mmu_hal_unmap_all();
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Eq, PartialEq, Debug)]
enum MmuPageSize {
    MMU_PAGE_8KB  = 0x2000,
    MMU_PAGE_16KB = 0x4000,
    MMU_PAGE_32KB = 0x8000,
    MMU_PAGE_64KB = 0x10000,
}

#[cfg(esp32s3)]
fn mmu_ll_set_page_size(mmu_id: u32, size: MmuPageSize) {
    assert_eq!(size, MmuPageSize::MMU_PAGE_64KB);
}

/// Unmap all the MMU table. After this all external memory vaddr are not
/// available
// NOTE: different for ESP32-P4
fn mmu_hal_unmap_all() {
    mmu_ll_unmap_all(0);
    // FIXME #if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    mmu_ll_unmap_all(1);
}

#[cfg(esp32s3)]
fn mmu_ll_unmap_all(mmu_id: u32) {
    const SOC_MMU_ENTRY_NUM: u32 = 512;

    for i in 0..SOC_MMU_ENTRY_NUM {
        mmu_ll_set_entry_invalid(mmu_id, i);
    }
}

#[cfg(esp32s3)]
fn mmu_ll_set_entry_invalid(mmu_id: u32, entry_id: u32) {
    const DR_REG_MMU_TABLE: u32 = 0x600C5000;
    const DR_REG_ITAG_TABLE: u32 = 0x600C6000;
    const DR_REG_DTAG_TABLE: u32 = 0x600C8000;
    const DR_REG_EXT_MEM_ENC: u32 = 0x600CC000;
    const SOC_MMU_INVALID: u32 = 1 << 14;
    const SOC_MMU_VALID: u32 = 0;
    unsafe {
        let mmu_table = (DR_REG_MMU_TABLE + entry_id * 4) as *mut u32;
        mmu_table.write_volatile(SOC_MMU_INVALID);
    }
}

const RTC_CNTL_FIB_GLITCH_RST: u32 = 1 << 0; // FIXME update pac
const BROWNOUT_DETECTOR_LL_FIB_ENABLE: u32 = 1 << 1;
const RTC_CNTL_FIB_SUPER_WDT_RST: u32 = 1 << 2;

#[cfg(esp32s3)]
fn bootloader_ana_super_wdt_reset_config() {
    clear_fib_sel_bit(RTC_CNTL_FIB_SUPER_WDT_RST);

    // enable super watchdog
    rtc_cntl()
        .swd_conf()
        .modify(|_, w| w.swd_bypass_rst().clear_bit());
}

/// Enable brownout hardware reset (mode1)
#[cfg(esp32s3)]
fn brownout_ll_ana_reset_enable() {
    clear_fib_sel_bit(BROWNOUT_DETECTOR_LL_FIB_ENABLE);
    // enable the BOD mode1 to reset the system
    rtc_cntl()
        .brown_out()
        .modify(|_, w| w.brown_out_ana_rst_en().set_bit());
}

#[cfg(esp32s3)]
fn bootloader_ana_clock_glitch_reset_config() {
    clear_fib_sel_bit(RTC_CNTL_FIB_GLITCH_RST);

    // enable a reset when the system detects a glitch
    rtc_cntl()
        .ana_conf()
        .modify(|_, w| w.glitch_rst_en().set_bit());
}

fn rtc_cntl() -> &'static <esp_hal::peripherals::LPWR as core::ops::Deref>::Target {
    unsafe { &*esp_hal::peripherals::LPWR::ptr() }
}

fn clear_fib_sel_bit(bit: u32) {
    unsafe {
        let rtc_cntl = rtc_cntl();
        rtc_cntl.fib_sel().modify(|r, w| w.bits(r.bits() & !bit));
    }
}

// https://www.elm-tech.com/en/products/spi-flash-memory/gd25q32/gd25q32.pdf
// MOSI [0x9F, 0x00, 0x00, 0x00]
// MISO [x, Manufacturer ID, Memory Type, Memory Capacity]
fn bootloader_read_flash_id() -> u32 {
    const CMD_RDID: u8 = 0x9f; // Read Identification
    let mut id = bootloader_execute_flash_command(CMD_RDID, 0, 0, 24);
    id = ((id & 0xff) << 16) | ((id >> 16) & 0xff) | (id & 0xff00);
    id
}

fn bootloader_execute_flash_command(
    command: u8,
    mosi_data: u32,
    mosi_len: u8,
    miso_len: u8,
) -> u32 {
    const addr_len: u8 = 0;
    const address: u8 = 0;
    const dummy_len: u8 = 0;

    bootloader_flash_execute_command_common(
        command, addr_len, address, dummy_len, mosi_len, mosi_data, miso_len,
    )
}

fn bootloader_flash_execute_command_common(
    command: u8,
    addr_len: u8,
    address: u8,
    dummy_len: u8,
    mosi_len: u8,
    mosi_data: u32,
    miso_len: u8,
) -> u32 {
    //#define SPIMEM_LL_APB SPIMEM1
    //#define SPIMEM_LL_CACHE SPIMEM0

    let spi = unsafe { esp_hal::peripherals::SPI1::steal() };
    // https://docs.esp-rs.org/esp-hal/esp-hal/0.21.0/esp32s3/esp_hal/spi/master/trait.HalfDuplexReadWrite.html
    // FIXME search for psram_exec_cmd

    // assert!(mosi_len <= 32);
    // assert!(miso_len <= 32);
    // let old_ctrl_reg = 0;
    // let old_user_reg = 0;
    // let old_user1_reg = 0;
    // let old_user2_reg = 0;
    // spi_flash_ll_get_common_command_register_info(
    // &SPIMEM_LL_APB,
    // &old_ctrl_reg,
    // &old_user_reg,
    // &old_user1_reg,
    // &old_user2_reg,
    // );
    // SPIMEM_LL_APB.ctrl.val = 0;
    // #[cfg(CONFIG_IDF_TARGET_ESP32)]
    // spi_flash_ll_set_wp_level(&SPIMEM_LL_APB, true);
    // #[cfg(not(CONFIG_IDF_TARGET_ESP32))]
    // spimem_flash_ll_set_wp_level(&SPIMEM_LL_APB, true);
    //
    // command phase
    // spi_flash_ll_set_command(&SPIMEM_LL_APB, command, 8);
    // addr phase
    // spi_flash_ll_set_addr_bitlen(&SPIMEM_LL_APB, addr_len);
    // spi_flash_ll_set_usr_address(&SPIMEM_LL_APB, address, addr_len);
    // dummy phase
    // let total_dummy = dummy_len
    // + if miso_len > 0 {
    // g_rom_spiflash_dummy_len_plus[1]
    // } else {
    // 0
    // };
    // spi_flash_ll_set_dummy(&SPIMEM_LL_APB, total_dummy);
    // output data
    // spi_flash_ll_set_mosi_bitlen(&SPIMEM_LL_APB, mosi_len);
    // spi_flash_ll_set_buffer_data(&SPIMEM_LL_APB, &mosi_data, mosi_len / 8);
    // input data
    // spi_flash_ll_set_miso_bitlen(&SPIMEM_LL_APB, miso_len);
    //
    // spi_flash_ll_user_start(&SPIMEM_LL_APB, false);
    // while !spi_flash_ll_cmd_is_done(&SPIMEM_LL_APB) {}
    //
    // spi_flash_ll_set_common_command_register_info(
    // &SPIMEM_LL_APB,
    // old_ctrl_reg,
    // old_user_reg,
    // old_user1_reg,
    // old_user2_reg,
    // );
    //
    // let mut output_data = 0;
    // spi_flash_ll_get_buffer_data(&SPIMEM_LL_APB, &mut output_data, miso_len /
    // 8); let mut ret = output_data;
    // if miso_len < 32 {
    // set unused bits to 0
    // ret &= !(u32::MAX << miso_len);
    // }
    // ret
    todo!()
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
