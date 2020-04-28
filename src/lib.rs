//! A platform agnostic Rust driver for the CS43L22, based on the
//! [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! TBD

#![no_std]
// #![deny(missing_docs)]

mod register;

// extern crate byteorder;
extern crate embedded_hal as hal;
use crate::register::{OutputDevice, Register};

// use byteorder::{BigEndian, ByteOrder};
// use hal::blocking::delay::DelayMs;
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Read, Write, WriteRead},
    digital::v2::OutputPin,
};

/// CS43L22 I2C address (shifted because addresses are 7 bit).
/// Assumes AD0 pin is low.
// pub const ADDRESS: u8 = 0x94 >> 1;
pub const ADDRESS: u8 = 0b100_1010;

/// CS43L22 device ID
/// NOTE: Not sure about this
pub const DEVICE_ID: u8 = 0xE0;
pub const DEVICE_ID_MASK: u8 = 0xF8;
// #define  CS43L22_ID_MASK       0xF8
// #define CS43L22_CHIPID_ADDR    0x01

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2C(E),
    /// Device invalid or other hardware error.
    Device,
    /// Device not initialized correctly.
    NotInitialized,
    NotReady,
}

/// Driver for the CS43L22 DAC
#[derive(Debug, Default)]
pub struct CS43L22<I2C, RESETPIN> {
    /// The concrete I²C device implementation.
    i2c: I2C,
    /// The reset pin to use during initialization.
    reset_pin: RESETPIN,
}

impl<I2C, I2CERR, RESETPIN, PINERR> CS43L22<I2C, RESETPIN>
where
    I2C: Read<Error = I2CERR> + Write<Error = I2CERR> + WriteRead<Error = I2CERR>,
    RESETPIN: OutputPin<Error = PINERR>,
{
    /// Initialize the CS43L22 driver.
    pub fn new<DELAY: DelayMs<u8>>(
        i2c: I2C,
        reset_pin: RESETPIN,
        delay: &mut DELAY,
        vol: u8,
    ) -> Result<Self, Error<I2CERR>> {
        let mut cs43l22 = CS43L22 { i2c, reset_pin };

        // TODO: Check real delay time, this uses the same as CMCIS examples.
        // let _ = cs43l22.reset_pin.set_low().and(Error::NotInitialized);
        let _ = cs43l22.reset_pin.set_low();
        let _ = delay.delay_ms(5);
        let _ = cs43l22.reset_pin.set_high();
        let _ = delay.delay_ms(5);

        // Ensure we have the correct device ID for the CS43L22.
        let id = cs43l22.get_device_id()?;
        if id != DEVICE_ID {
            return Err(Error::Device);
        }

        // Keep powered OFF.
        cs43l22.write_reg(Register::PowerCtl1, 0x01)?;

        // Initialization settings?
        cs43l22.write_reg(0x00, 0x99)?;
        cs43l22.write_reg(0x47, 0x80)?;
        // let d32 = cs43l22.read_reg(0x32)?;
        // cs43l22.write_reg(0x32, d32 | 0b1000_0000)?;
        // cs43l22.write_reg(0x32, d32 & 0b0111_1111)?;
        cs43l22.write_reg(0x32, 0x80)?;
        cs43l22.write_reg(0x32, 0x00)?;
        cs43l22.write_reg(0x00, 0x00)?;

        // SPK always OFF & HP always ON.
        cs43l22.set_output(OutputDevice::Headphone)?;
        // Clock configuration: Auto detection and ~~divide MCLK by 2~~.
        // cs43l22.write_reg(Register::ClockingCtl, 0x81)?;
        // cs43l22.write_reg(Register::ClockingCtl, (1 << 7))?;
        // cs43l22.write_reg(Register::ClockingCtl, (1 << 7) | (1 << 0))?;
        // Set Slave Mode and interface format (Philips/I2S).
        // cs43l22.write_reg(Register::InterfaceCtl1, 0x04)?;
        // cs43l22.write_reg(Register::InterfaceCtl1, 0b01 << 3)?;

        // Set volume.
        cs43l22.set_vol(vol)?;

        // Power ON.
        cs43l22.write_reg(Register::PowerCtl1, 0x9E)?;

        /* Additional configuration for the CODEC. These configurations are done to reduce
        the time needed for the Codec to power off. If these configurations are removed,
        then a long delay should be added between powering off the Codec and switching
        off the I2S peripheral MCLK clock (which is the operating clock for Codec).
        If this delay is not inserted, then the codec will not shut down properly and
        it results in high noise after shut down. */

        /* Disable the analog soft ramp */
        cs43l22.write_reg(Register::AnalogZCAndSRSettings, 0x00)?;
        /* Disable the digital soft ramp */
        cs43l22.write_reg(Register::Misc, 0x04)?;
        /* Disable the limiter attack level */
        cs43l22.write_reg(Register::LimitCtl1AndThresholds, 0x00)?;
        /* Adjust Bass and Treble levels */
        cs43l22.write_reg(Register::ToneCtl, 0x0F)?;
        /* Adjust PCM volume level */
        cs43l22.write_reg(Register::PCMAVol, 0x0A)?;
        cs43l22.write_reg(Register::PCMBVol, 0x0A)?;

        /* Configure the I2S peripheral */
        // JK: This should be done by the Hal. It's just the setup of the I2S peripheral
        //     But the settings needs to be known by the codec.
        //TODO: Codec_AudioInterface_Init(AudioFreq);

        Ok(cs43l22)
    }

    /// Get the device ID.
    fn get_device_id(&mut self) -> Result<u8, Error<I2CERR>> {
        self.read_reg(Register::DeviceID)
            .map(|id| id & DEVICE_ID_MASK)
    }

    /// Write register.
    pub fn write_reg<T: Into<u8>, V: Into<u8>>(
        &mut self,
        addr: T,
        value: V,
    ) -> Result<(), Error<I2CERR>> {
        self.i2c
            .write(ADDRESS, &[addr.into(), value.into()])
            .map_err(Error::I2C)
    }

    /// Read register.
    pub fn read_reg<T: Into<u8>>(&mut self, addr: T) -> Result<u8, Error<I2CERR>> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(ADDRESS, &[addr.into()], &mut buf)
            .map_err(Error::I2C)
            .and(Ok(buf[0]))
    }

    /// Sets the master volume for A and B together.
    /// The range is -102dB to +12dB with 0.5dB increments represented as
    /// 0-255 with 0dB at 231.
    pub fn set_vol(&mut self, vol: u8) -> Result<(), Error<I2CERR>> {
        if vol > 0xE6 {
            /* Set the Master volume */
            self.write_reg(Register::MasterAVol, vol - 0xE7)?;
            self.write_reg(Register::MasterBVol, vol - 0xE7)?;
        } else {
            /* Set the Master volume */
            self.write_reg(Register::MasterAVol, vol + 0x19)?;
            self.write_reg(Register::MasterBVol, vol + 0x19)?;
        }
        Ok(())
    }

    /// Sets the output device. Currently only a subset is supported.
    pub fn set_output(&mut self, dev: OutputDevice) -> Result<(), Error<I2CERR>> {
        self.write_reg(Register::PowerCtl2, dev)
    }

    pub fn beep(&mut self) -> Result<(), Error<I2CERR>> {
        self.write_reg(Register::BeepAndToneCfg, 0b10 << 6)
    }
}
