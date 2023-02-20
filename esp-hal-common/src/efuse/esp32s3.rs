//! Reading of eFuses

use crate::{analog::adc::Attenuation, peripherals::EFUSE};

pub struct Efuse;

impl Efuse {
    /// Reads chip's MAC address from the eFuse storage.
    ///
    /// # Example
    ///
    /// ```
    /// let mac_address = Efuse::get_mac_address();
    /// writeln!(
    ///     serial_tx,
    ///     "MAC: {:#X}:{:#X}:{:#X}:{:#X}:{:#X}:{:#X}",
    ///     mac_address[0],
    ///     mac_address[1],
    ///     mac_address[2],
    ///     mac_address[3],
    ///     mac_address[4],
    ///     mac_address[5]
    /// );
    /// ```
    pub fn get_mac_address() -> [u8; 6] {
        let efuse = unsafe { &*EFUSE::ptr() };

        let mac_low: u32 = efuse.rd_mac_spi_sys_0.read().mac_0().bits();
        let mac_high: u32 = efuse.rd_mac_spi_sys_1.read().mac_1().bits() as u32;

        let mac_low_bytes = mac_low.to_be_bytes();
        let mac_high_bytes = mac_high.to_be_bytes();

        [
            mac_high_bytes[2],
            mac_high_bytes[3],
            mac_low_bytes[0],
            mac_low_bytes[1],
            mac_low_bytes[2],
            mac_low_bytes[3],
        ]
    }

    /// Get status of SPI boot encryption.
    pub fn get_flash_encryption() -> bool {
        let efuse = unsafe { &*EFUSE::ptr() };
        (efuse
            .rd_repeat_data1
            .read()
            .spi_boot_crypt_cnt()
            .bits()
            .count_ones()
            % 2)
            != 0
    }

    /// Get the multiplier for the timeout value of the RWDT STAGE 0 register.
    pub fn get_rwdt_multiplier() -> u8 {
        let efuse = unsafe { &*EFUSE::ptr() };
        efuse.rd_repeat_data1.read().wdt_delay_sel().bits()
    }

    pub fn adc1_get_init_code(atten: Attenuation) -> u32 {
        // logic taken from https://github.com/espressif/esp-idf/blob/7052a14d61d98dd387c4d99f435c42370b282fa6/components/efuse/esp32s3/esp_efuse_rtc_calib.c#L28-L61
        assert!(EFuseAddress::BLK_VERSION_MAJOR.read() == 1);

        let diffs = [
            EFuseAddress::ADC1_INIT_CODE_ATTEN0.read(),
            EFuseAddress::ADC1_INIT_CODE_ATTEN1.read(),
            EFuseAddress::ADC1_INIT_CODE_ATTEN2.read(),
            EFuseAddress::ADC1_INIT_CODE_ATTEN3.read(),
        ];

        let mut init_codes = [0, 0, 0, 0];
        init_codes[0] = diffs[0] + 1850;
        init_codes[1] = init_codes[0] + diffs[1] + 90;
        init_codes[2] = init_codes[1] + diffs[2];
        init_codes[3] = init_codes[2] + diffs[3] + 70;

        match atten {
            Attenuation::Attenuation0dB => init_codes[0],
            Attenuation::Attenuation2p5dB => init_codes[1],
            Attenuation::Attenuation6dB => init_codes[2],
            Attenuation::Attenuation11dB => init_codes[3],
        }
    }

    pub fn adc2_get_init_code(atten: Attenuation) -> u32 {
        // logic taken from https://github.com/espressif/esp-idf/blob/7052a14d61d98dd387c4d99f435c42370b282fa6/components/efuse/esp32s3/esp_efuse_rtc_calib.c#L28-L61
        assert!(EFuseAddress::BLK_VERSION_MAJOR.read() == 1);

        let diffs = [
            EFuseAddress::ADC2_INIT_CODE_ATTEN0.read(),
            EFuseAddress::ADC2_INIT_CODE_ATTEN1.read(),
            EFuseAddress::ADC2_INIT_CODE_ATTEN2.read(),
            EFuseAddress::ADC2_INIT_CODE_ATTEN3.read(),
        ];

        let mut init_codes = [0, 0, 0, 0];
        init_codes[0] = diffs[0] + 2020;
        init_codes[1] = init_codes[0] + diffs[1];
        init_codes[2] = init_codes[1] + diffs[2];
        init_codes[3] = init_codes[2] + diffs[3];

        match atten {
            Attenuation::Attenuation0dB => init_codes[0],
            Attenuation::Attenuation2p5dB => init_codes[1],
            Attenuation::Attenuation6dB => init_codes[2],
            Attenuation::Attenuation11dB => init_codes[3],
        }
    }

    pub fn adc1_get_cal_voltage(atten: Attenuation) -> (u32, u32) {
        // logic taken from https://github.com/espressif/esp-idf/blob/7052a14d61d98dd387c4d99f435c42370b282fa6/components/efuse/esp32s3/esp_efuse_rtc_calib.c#L63-L93
        assert!(EFuseAddress::BLK_VERSION_MAJOR.read() == 1);

        let diffs = [
            EFuseAddress::ADC1_CAL_VOL_ATTEN0.read(),
            EFuseAddress::ADC1_CAL_VOL_ATTEN1.read(),
            EFuseAddress::ADC1_CAL_VOL_ATTEN2.read(),
            EFuseAddress::ADC1_CAL_VOL_ATTEN3.read(),
        ];

        let mut voltages = [0, 0, 0, 0];
        voltages[3] = diffs[3] + 900;
        voltages[2] = voltages[3] + diffs[2] + 800;
        voltages[1] = voltages[2] + diffs[1] + 700;
        voltages[0] = voltages[1] + diffs[0] + 800;

        match atten {
            Attenuation::Attenuation0dB => (voltages[0], 850),
            Attenuation::Attenuation2p5dB => (voltages[1], 850),
            Attenuation::Attenuation6dB => (voltages[2], 850),
            Attenuation::Attenuation11dB => (voltages[3], 850),
        }
    }

    pub fn adc2_get_cal_voltage(atten: Attenuation) -> (u32, u32) {
        // logic taken from https://github.com/espressif/esp-idf/blob/7052a14d61d98dd387c4d99f435c42370b282fa6/components/efuse/esp32s3/esp_efuse_rtc_calib.c#L63-L93
        assert!(EFuseAddress::BLK_VERSION_MAJOR.read() == 1);

        let diffs = [
            EFuseAddress::ADC1_CAL_VOL_ATTEN0.read(),
            EFuseAddress::ADC1_CAL_VOL_ATTEN1.read(),
            EFuseAddress::ADC1_CAL_VOL_ATTEN2.read(),
            EFuseAddress::ADC1_CAL_VOL_ATTEN3.read(),
            EFuseAddress::ADC2_CAL_VOL_ATTEN0.read(),
            EFuseAddress::ADC2_CAL_VOL_ATTEN1.read(),
            EFuseAddress::ADC2_CAL_VOL_ATTEN2.read(),
            EFuseAddress::ADC2_CAL_VOL_ATTEN3.read(),
        ];

        let mut voltages = [0, 0, 0, 0, 0, 0, 0, 0];
        voltages[3] = diffs[3] + 900;
        voltages[2] = voltages[3] + diffs[2] + 800;
        voltages[1] = voltages[2] + diffs[1] + 700;
        voltages[0] = voltages[1] + diffs[0] + 800;
        voltages[7] = voltages[3] - diffs[7] + 15;
        voltages[6] = voltages[2] - diffs[6] + 20;
        voltages[5] = voltages[1] - diffs[5] + 10;
        voltages[4] = voltages[0] - diffs[4] + 40;

        match atten {
            Attenuation::Attenuation0dB => (voltages[4], 850),
            Attenuation::Attenuation2p5dB => (voltages[5], 850),
            Attenuation::Attenuation6dB => (voltages[6], 850),
            Attenuation::Attenuation11dB => (voltages[7], 850),
        }
    }
}

#[allow(unused)]
#[derive(Copy, Clone)]
enum EFuseBlock {
    Block0,
    Block1,
    Block2,
    Block3,
    Block4,
    Block5,
    Block6,
    Block7,
    Block8,
    Block9,
    Block10,
}

impl EFuseBlock {
    fn address(self) -> *const u32 {
        let efuse = unsafe { &*EFUSE::ptr() };
        match self {
            EFuseBlock::Block0 => efuse.rd_wr_dis.as_ptr(),
            EFuseBlock::Block1 => efuse.rd_mac_spi_sys_0.as_ptr(),
            EFuseBlock::Block2 => efuse.rd_sys_part1_data0.as_ptr(),
            EFuseBlock::Block3 => efuse.rd_usr_data0.as_ptr(),
            EFuseBlock::Block4 => efuse.rd_key0_data0.as_ptr(),
            EFuseBlock::Block5 => efuse.rd_key1_data0.as_ptr(),
            EFuseBlock::Block6 => efuse.rd_key2_data0.as_ptr(),
            EFuseBlock::Block7 => efuse.rd_key3_data0.as_ptr(),
            EFuseBlock::Block8 => efuse.rd_key4_data0.as_ptr(),
            EFuseBlock::Block9 => efuse.rd_key5_data0.as_ptr(),
            EFuseBlock::Block10 => efuse.rd_sys_part2_data0.as_ptr(),
        }
    }
}

pub struct EFuseAddress {
    block: EFuseBlock,
    offset: u8,
    size: u8,
}

impl EFuseAddress {
    pub fn read(&self) -> u32 {
        let first_word_address = unsafe { self.block.address().add(self.offset as usize / 32) };
        let first_word: u32 = unsafe { first_word_address.read_volatile() };

        let offset_in_word = self.offset % 32;
        let mask = u32::MAX >> (32 - self.size);

        if offset_in_word + self.size > 32 {
            let second_word: u32 = unsafe { first_word_address.add(1).read_volatile() };
            ((first_word >> offset_in_word) | (second_word << (32 - offset_in_word))) & mask
        } else {
            (first_word >> offset_in_word) & mask
        }
    }

    // taken from https://github.com/espressif/esp-idf/blob/045163a2ec99eb3cb7cc69e2763afd145156c4cf/components/efuse/esp32s3/esp_efuse_table.csv

    /// BLK_VERSION_MAJOR of BLOCK2 change of this bit means users need to
    /// update firmware
    pub const BLK_VERSION_MAJOR: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 128,
        size: 2,
    };

    /// ADC1 init code at atten0
    pub const ADC1_INIT_CODE_ATTEN0: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 149,
        size: 8,
    };
    /// ADC1 init code at atten1
    pub const ADC1_INIT_CODE_ATTEN1: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 157,
        size: 6,
    };
    /// ADC1 init code at atten2
    pub const ADC1_INIT_CODE_ATTEN2: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 163,
        size: 6,
    };
    /// ADC1 init code at atten3
    pub const ADC1_INIT_CODE_ATTEN3: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 169,
        size: 6,
    };
    /// ADC2 init code at atten0
    pub const ADC2_INIT_CODE_ATTEN0: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 175,
        size: 8,
    };
    /// ADC2 init code at atten1
    pub const ADC2_INIT_CODE_ATTEN1: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 183,
        size: 6,
    };
    /// ADC2 init code at atten2
    pub const ADC2_INIT_CODE_ATTEN2: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 189,
        size: 6,
    };
    /// ADC2 init code at atten3
    pub const ADC2_INIT_CODE_ATTEN3: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 195,
        size: 6,
    };
    /// ADC1 calibration voltage at atten0
    pub const ADC1_CAL_VOL_ATTEN0: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 201,
        size: 8,
    };
    /// ADC1 calibration voltage at atten1
    pub const ADC1_CAL_VOL_ATTEN1: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 209,
        size: 8,
    };
    /// ADC1 calibration voltage at atten2
    pub const ADC1_CAL_VOL_ATTEN2: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 217,
        size: 8,
    };
    /// ADC1 calibration voltage at atten3
    pub const ADC1_CAL_VOL_ATTEN3: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 225,
        size: 8,
    };
    /// ADC2 calibration voltage at atten0
    pub const ADC2_CAL_VOL_ATTEN0: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 233,
        size: 8,
    };
    /// ADC2 calibration voltage at atten1
    pub const ADC2_CAL_VOL_ATTEN1: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 241,
        size: 7,
    };
    /// ADC2 calibration voltage at atten2
    pub const ADC2_CAL_VOL_ATTEN2: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block2,
        offset: 248,
        size: 7,
    };
    /// ADC2 calibration voltage at atten3
    pub const ADC2_CAL_VOL_ATTEN3: EFuseAddress = EFuseAddress {
        block: EFuseBlock::Block1,
        offset: 186,
        size: 6,
    };
}
