use core::marker::PhantomData;

use embedded_hal::adc::{Channel, OneShot};

use crate::{
    analog::{ADC1, ADC2},
    peripheral::PeripheralRef,
    peripherals::{APB_SARADC, SENS},
};

/// The sampling/readout resolution of the ADC
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Resolution {
    Resolution13Bit,
}

/// The attenuation of the ADC pin
#[derive(PartialEq, Eq, Clone, Copy)]
pub enum Attenuation {
    Attenuation0dB   = 0b00,
    Attenuation2p5dB = 0b01,
    Attenuation6dB   = 0b10,
    Attenuation11dB  = 0b11,
}

pub struct AdcPin<PIN, ADCI> {
    pub pin: PIN,
    _phantom: PhantomData<ADCI>,
}

impl<PIN: Channel<ADCI, ID = u8>, ADCI> Channel<ADCI> for AdcPin<PIN, ADCI> {
    type ID = u8;

    fn channel() -> Self::ID {
        PIN::channel()
    }
}

pub struct AdcConfig<ADCI> {
    pub resolution: Resolution,
    pub attenuations: [Option<Attenuation>; 10],
    _phantom: PhantomData<ADCI>,
}

impl<ADCI> AdcConfig<ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn new() -> AdcConfig<ADCI> {
        Self::default()
    }

    pub fn enable_pin<PIN: Channel<ADCI, ID = u8>>(
        &mut self,
        pin: PIN,
        attenuation: Attenuation,
    ) -> AdcPin<PIN, ADCI> {
        self.attenuations[PIN::channel() as usize] = Some(attenuation);

        AdcPin {
            pin,
            _phantom: PhantomData::default(),
        }
    }
}

impl<ADCI> Default for AdcConfig<ADCI> {
    fn default() -> Self {
        AdcConfig {
            resolution: Resolution::Resolution13Bit,
            attenuations: [None; 10],
            _phantom: PhantomData::default(),
        }
    }
}

#[doc(hidden)]
pub trait RegisterAccess {
    fn set_attenuation(channel: usize, attenuation: u8);

    fn clear_dig_force();

    fn set_start_force();

    fn set_en_pad_force();

    fn start_sar(channel: u8);

    fn read_data_sar() -> Option<u16>;
}

impl RegisterAccess for ADC1 {
    fn set_attenuation(channel: usize, attenuation: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_atten1.modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation as u8 & 0b11) as u32) << (channel * 2));

            unsafe { w.sar1_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_mux
            .modify(|_, w| w.sar1_dig_force().clear_bit());
    }

    fn set_start_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2
            .modify(|_, w| w.meas1_start_force().set_bit());
    }

    fn set_en_pad_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas1_ctrl2
            .modify(|_, w| w.sar1_en_pad_force().set_bit());
    }

    fn start_sar(channel: u8) {
        let sensors = unsafe { &*SENS::ptr() };

        sensors.sar_meas1_ctrl2.modify(|_, w| unsafe {
            w.sar1_en_pad()
                .bits(1 << channel)
                .meas1_start_sar()
                .clear_bit()
        });

        sensors
            .sar_meas1_ctrl2
            .modify(|_, w| w.meas1_start_sar().set_bit());
    }

    #[inline(always)]
    fn read_data_sar() -> Option<u16> {
        let sensors = unsafe { &*SENS::ptr() };
        let reg = sensors.sar_meas1_ctrl2.read();
        if reg.meas1_done_sar().bit_is_set() {
            Some(reg.meas1_data_sar().bits())
        } else {
            None
        }
    }
}

impl RegisterAccess for ADC2 {
    fn set_attenuation(channel: usize, attenuation: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_atten2.modify(|r, w| {
            let new_value = (r.bits() & !(0b11 << (channel * 2)))
                | (((attenuation as u8 & 0b11) as u32) << (channel * 2));

            unsafe { w.sar2_atten().bits(new_value) }
        });
    }

    fn clear_dig_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_mux
            .modify(|_, w| w.sar2_rtc_force().set_bit());

        let sar_apb = unsafe { &*APB_SARADC::ptr() };
        sar_apb
            .arb_ctrl
            .modify(|_, w| w.adc_arb_rtc_force().set_bit());
    }

    fn set_start_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2
            .modify(|_, w| w.meas2_start_force().set_bit());
    }

    fn set_en_pad_force() {
        let sensors = unsafe { &*SENS::ptr() };
        sensors
            .sar_meas2_ctrl2
            .modify(|_, w| w.sar2_en_pad_force().set_bit());
    }

    fn start_sar(channel: u8) {
        let sensors = unsafe { &*SENS::ptr() };
        sensors.sar_meas2_ctrl2.modify(|_, w| unsafe {
            w.sar2_en_pad()
                .bits(1 << channel)
                .meas2_start_sar()
                .clear_bit()
        });

        sensors
            .sar_meas2_ctrl2
            .modify(|_, w| w.meas2_start_sar().set_bit());
    }

    #[inline(always)]
    fn read_data_sar() -> Option<u16> {
        let sensors = unsafe { &*SENS::ptr() };
        let reg = sensors.sar_meas2_ctrl2.read();
        if reg.meas2_done_sar().bit_is_set() {
            Some(reg.meas2_data_sar().bits())
        } else {
            None
        }
    }
}

pub struct ADC<'d, ADC> {
    _adc: PeripheralRef<'d, ADC>,
    attenuations: [Option<Attenuation>; 10],
    active_channel: Option<u8>,
}

impl<'d, ADCI> ADC<'d, ADCI>
where
    ADCI: RegisterAccess,
{
    pub fn adc(
        adc_instance: impl crate::peripheral::Peripheral<P = ADCI> + 'd,
        config: AdcConfig<ADCI>,
    ) -> Result<Self, ()> {
        let sensors = unsafe { &*SENS::ptr() };

        // Set attenuation for pins
        let attenuations = config.attenuations;

        for channel in 0..attenuations.len() {
            if let Some(attenuation) = attenuations[channel] {
                ADCI::set_attenuation(channel, attenuation as u8);
            }
        }

        // SAR ADCI controlled by DIG ADC1 controller.
        ADCI::clear_dig_force();
        // RTC ADCI controller is started by software
        ADCI::set_start_force();
        // SAR ADCI pin enable bitmap is controlled by software
        ADCI::set_en_pad_force();

        // Enable clock
        #[cfg(esp32s2)]
        sensors
            .sar_meas1_ctrl1
            .modify(|_, w| w.rtc_saradc_clkgate_en().set_bit());

        #[cfg(esp32s3)]
        sensors
            .sar_peri_clk_gate_conf
            .modify(|_, w| w.saradc_clk_en().set_bit());

        let adc = ADC {
            _adc: adc_instance.into_ref(),
            attenuations: config.attenuations,
            active_channel: None,
        };

        Ok(adc)
    }

    pub fn read_blocking<PIN: Channel<ADCI, ID = u8>>(
        &mut self,
        _pin: &mut AdcPin<PIN, ADCI>,
    ) -> u16 {
        ADCI::start_sar(AdcPin::<PIN, ADCI>::channel() as u8);

        loop {
            if let Some(data) = ADCI::read_data_sar() {
                break data;
            }
        }
    }
}

impl<'d, ADCI, WORD, PIN> OneShot<ADCI, WORD, AdcPin<PIN, ADCI>> for ADC<'d, ADCI>
where
    WORD: From<u16>,
    PIN: Channel<ADCI, ID = u8>,
    ADCI: RegisterAccess,
{
    type Error = ();

    fn read(&mut self, _pin: &mut AdcPin<PIN, ADCI>) -> nb::Result<WORD, Self::Error> {
        if self.attenuations[AdcPin::<PIN, ADCI>::channel() as usize] == None {
            panic!(
                "Channel {} is not configured reading!",
                AdcPin::<PIN, ADCI>::channel()
            );
        }

        if let Some(active_channel) = self.active_channel {
            // There is conversion in progress:
            // - if it's for a different channel try again later
            // - if it's for the given channel, go ahead and check progress
            if active_channel != AdcPin::<PIN, ADCI>::channel() {
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // If no conversions are in progress, start a new one for given channel
            self.active_channel = Some(AdcPin::<PIN, ADCI>::channel());

            ADCI::start_sar(AdcPin::<PIN, ADCI>::channel() as u8);
        }

        // Wait for ADC to finish conversion
        match ADCI::read_data_sar() {
            None => Err(nb::Error::WouldBlock),
            Some(converted_value) => {
                // Mark that no conversions are currently in progress
                self.active_channel = None;
                Ok(converted_value.into())
            }
        }
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_adc_interface {
    ($adc:ident [
        $( ($pin:ident, $channel:expr) ,)+
    ]) => {

        $(
            impl Channel<$adc> for $pin<Analog> {
                type ID = u8;

                fn channel() -> u8 { $channel }
            }
        )+
    }
}

pub use impl_adc_interface;

#[cfg(esp32s3)]
pub mod implementation {
    //! Analog to digital (ADC) conversion support.
    //!
    //! This module provides functions for reading analog values from two
    //! analog to digital converters available on the ESP32-S3: `ADC1` and
    //! `ADC2`.

    use embedded_hal::adc::Channel;
    use paste::paste;

    use super::impl_adc_interface;
    pub use crate::analog::{adc::*, ADC1, ADC2};
    use crate::{gpio::*, regi2c_write_mask, rom::regi2c_ctrl_write_reg_mask};

    impl_adc_interface! {
        ADC1 [
            (Gpio1, 0),
            (Gpio2, 1),
            (Gpio3, 2),
            (Gpio4, 3),
            (Gpio5, 4),
            (Gpio6, 5),
            (Gpio7, 6),
            (Gpio8, 7),
            (Gpio9, 8),
            (Gpio10,9),
        ]
    }

    impl_adc_interface! {
        ADC2 [
            (Gpio11, 0),
            (Gpio12, 1),
            (Gpio13, 2),
            (Gpio14, 3),
            (Gpio15, 4),
            (Gpio16, 5),
            (Gpio17, 6),
            (Gpio18, 7),
            (Gpio19, 8),
            (Gpio20, 9),
        ]
    }

    // constants taken from https://github.com/espressif/esp-idf/blob/045163a2ec99eb3cb7cc69e2763afd145156c4cf/components/soc/esp32s3/include/soc/regi2c_saradc.h
    const I2C_SAR_ADC: u32 = 0x69;
    const I2C_SAR_ADC_HOSTID: u32 = 1;

    const ADC_SAR1_ENCAL_GND_ADDR: u32 = 0x7;
    const ADC_SAR1_ENCAL_GND_ADDR_MSB: u32 = 5;
    const ADC_SAR1_ENCAL_GND_ADDR_LSB: u32 = 5;

    const ADC_SAR2_ENCAL_GND_ADDR: u32 = 0x7;
    const ADC_SAR2_ENCAL_GND_ADDR_MSB: u32 = 7;
    const ADC_SAR2_ENCAL_GND_ADDR_LSB: u32 = 7;

    const ADC_SAR1_INITIAL_CODE_HIGH_ADDR: u32 = 0x1;
    const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_MSB: u32 = 0x3;
    const ADC_SAR1_INITIAL_CODE_HIGH_ADDR_LSB: u32 = 0x0;

    const ADC_SAR1_INITIAL_CODE_LOW_ADDR: u32 = 0x0;
    const ADC_SAR1_INITIAL_CODE_LOW_ADDR_MSB: u32 = 0x7;
    const ADC_SAR1_INITIAL_CODE_LOW_ADDR_LSB: u32 = 0x0;

    const ADC_SAR2_INITIAL_CODE_HIGH_ADDR: u32 = 0x4;
    const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_MSB: u32 = 0x3;
    const ADC_SAR2_INITIAL_CODE_HIGH_ADDR_LSB: u32 = 0x0;

    const ADC_SAR2_INITIAL_CODE_LOW_ADDR: u32 = 0x3;
    const ADC_SAR2_INITIAL_CODE_LOW_ADDR_MSB: u32 = 0x7;
    const ADC_SAR2_INITIAL_CODE_LOW_ADDR_LSB: u32 = 0x0;

    const ADC_SAR1_DREF_ADDR: u32 = 0x2;
    const ADC_SAR1_DREF_ADDR_MSB: u32 = 0x6;
    const ADC_SAR1_DREF_ADDR_LSB: u32 = 0x4;

    const ADC_SAR2_DREF_ADDR: u32 = 0x5;
    const ADC_SAR2_DREF_ADDR_MSB: u32 = 0x6;
    const ADC_SAR2_DREF_ADDR_LSB: u32 = 0x4;

    const _ADC_SAR1_SAMPLE_CYCLE_ADDR: u32 = 0x2;
    const _ADC_SAR1_SAMPLE_CYCLE_ADDR_MSB: u32 = 0x2;
    const _ADC_SAR1_SAMPLE_CYCLE_ADDR_LSB: u32 = 0x0;

    const _ADC_SARADC_DTEST_RTC_ADDR: u32 = 0x7;
    const _ADC_SARADC_DTEST_RTC_ADDR_MSB: u32 = 1;
    const _ADC_SARADC_DTEST_RTC_ADDR_LSB: u32 = 0;

    const _ADC_SARADC_ENT_TSENS_ADDR: u32 = 0x7;
    const _ADC_SARADC_ENT_TSENS_ADDR_MSB: u32 = 2;
    const _ADC_SARADC_ENT_TSENS_ADDR_LSB: u32 = 2;

    const _ADC_SARADC_ENT_RTC_ADDR: u32 = 0x7;
    const _ADC_SARADC_ENT_RTC_ADDR_MSB: u32 = 3;
    const _ADC_SARADC_ENT_RTC_ADDR_LSB: u32 = 3;

    const _ADC_SARADC_ENCAL_REF_ADDR: u32 = 0x7;
    const _ADC_SARADC_ENCAL_REF_ADDR_MSB: u32 = 4;
    const _ADC_SARADC_ENCAL_REF_ADDR_LSB: u32 = 4;

    const _I2C_SARADC_TSENS_DAC: u32 = 0x6;
    const _I2C_SARADC_TSENS_DAC_MSB: u32 = 3;
    const _I2C_SARADC_TSENS_DAC_LSB: u32 = 0;

    pub fn adc1_set_calibration(val: u16) {
        let [val_h, val_l] = val.to_be_bytes();

        unsafe {
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_INITIAL_CODE_HIGH_ADDR, val_h as u32);
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_INITIAL_CODE_LOW_ADDR, val_l as u32);
        }
    }
    pub fn adc2_set_calibration(val: u16) {
        let [val_h, val_l] = val.to_be_bytes();

        unsafe {
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_INITIAL_CODE_HIGH_ADDR, val_h as u32);
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_INITIAL_CODE_LOW_ADDR, val_l as u32);
        }
    }

    /// Enable/disable internal connect GND (for calibration).
    pub fn adc1_set_gnd_connect(enable: bool) {
        unsafe {
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_ENCAL_GND_ADDR, enable as u32);
        }
    }
    /// Enable/disable internal connect GND (for calibration).
    pub fn adc2_set_gnd_connect(enable: bool) {
        unsafe {
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_ENCAL_GND_ADDR, enable as u32);
        }
    }
    /// Set common calibration configuration
    pub fn adc1_calibration_init() {
        unsafe {
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR1_DREF_ADDR, 4);
        }
    }
    /// Set common calibration configuration
    pub fn adc2_calibration_init() {
        unsafe {
            regi2c_write_mask!(I2C_SAR_ADC, ADC_SAR2_DREF_ADDR, 4);
        }
    }
}
