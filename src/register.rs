//! CS43L22 register addresses

/// Register addresses
/// Taken from the ADXL343 data sheet (Register Map, p.21)
/// <https://www.analog.com/media/en/technical-documentation/data-sheets/adxl343.pdf>
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum Register {
    /// Device ID (Read Only)
    ///
    /// "The DeviceID register holds a fixed device ID code of 0x01."
    DeviceID = 0x01,

    PowerCtl1 = 0x02,
    // 0x03 - Reserved
    PowerCtl2 = 0x04,
    ClockingCtl = 0x05,
    InterfaceCtl1 = 0x06,
    InterfaceCtl2 = 0x07,
    PassthroughASelect = 0x08,
    PassthroughBSelect = 0x09,
    AnalogZCAndSRSettings = 0x0A,
    // 0x0B - Reserved
    PassthroughGangControl = 0x0C,
    PlaybackCtl1 = 0x0D,
    Misc = 0x0E,
    PlaybackCtl2 = 0x0F,
    // 0x10-0x13 - Reserved
    PassthroughA = 0x14,
    PassthroughB = 0x15,
    // 0x16-0x19 - Reserved
    PCMAVol = 0x1A,
    PCMBVol = 0x1B,
    BeepFreqOnTime = 0x1C,
    BeepVolOffTime = 0x1D,
    BeepAndToneCfg = 0x1E,
    ToneCtl = 0x1F,
    MasterAVol = 0x20,
    MasterBVol = 0x21,
    HeadphoneAVol = 0x22,
    HeadphoneBVol = 0x23,
    SpeakerAVol = 0x24,
    SpeakerBVol = 0x25,
    ChannelMixerAndSwap = 0x26,
    LimitCtl1AndThresholds = 0x27,
    LimitCtl2AndReleaseRate = 0x28,
    LimiterAttackRate = 0x29,
    // 0x2A-0x2D - Reserved
    OverflowAndClockStatus = 0x2E,
    BatteryCompensation = 0x2F,
    VPBatteryLevel = 0x30,
    SpeakerStatus = 0x31,
    // 0x32-0x33 - Reserved
    ChargePumpFreq = 0x34,
}

impl Into<u8> for Register {
    fn into(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
// TODO: Define a better type for this to support all combinations.
pub enum OutputDevice {
    Speaker = 0xFA,
    Headphone = 0xAF,
    Both = 0xAA,
    Auto = 0x05,
}

impl Into<u8> for OutputDevice {
    fn into(self) -> u8 {
        self as u8
    }
}
