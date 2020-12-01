#![allow(unused)]
use core::mem::size_of;
use alloc::vec::Vec;

const READ_FLAG: u8 = 0x80;
const WRITE_FLAG: u8 = 0x7F;

pub trait Register {
    //addr returns list of addresses that this register contains
    fn first_addr(&self) -> u8;
    //returns array with first addr of Register, followed by 0x00 * len of register
    fn read(&self) -> Vec<u8>;
    fn write(&self) -> Vec<u8>;

    fn new(values: &[u8]) -> Self;
}

macro_rules! create_struct {
    // input is empty: time to output
    (@inner () -> {struct $name:ident $(($id:ident: $ty:ty))*}) => {
        struct $name { $($id: $ty),* }
    };

    // throw on the last field
    (@inner ($id:ident: $no: expr) -> {$($output:tt)*}) => {
        create_struct!(@inner () -> {$($output)* ($id: [bool; $no])});
    };
      // throw on the last field
    (@inner ($id:ident) -> {$($output:tt)*}) => {
        create_struct!(@inner () -> {$($output)* ($id: bool)});
    };

    // throw on another field (not the last one)
    (@inner ($id:ident: $no:expr, $($next:tt)*) -> {$($output:tt)*}) => {
        create_struct!(@inner ($($next)*) -> {$($output)* ($id: [bool; $no])});
    };
    // throw on another field (not the last one)
    (@inner ($id:ident, $($next:tt)*) -> {$($output:tt)*}) => {
        create_struct!(@inner ($($next)*) -> {$($output)* ($id: bool)});
    };

    // entry point
    ($name:ident, $tp:ty) => {


    }
}
///create_struct!(@inner ($($input)*) -> {struct $name});

macro_rules! create_register {
    // entry point
    ($name: ident, u8[$addr0: expr], [$($output:tt)*]) => {
        pub struct $name(u8);
        impl Register for $name {
            fn first_addr(&self) -> u8 {
                $addr0
            }
            fn read(&self) -> Vec<u8> {
                vec![$addr0 | READ_FLAG, 0x00]
            }
            fn write(&self) -> Vec<u8> {
                vec![$addr0 & WRITE_FLAG, self.0]
            }
            fn new(values: &[u8]) -> Self {
                Self(values[0])
            }
        }
    };
    ($name: ident, u16[$addr0: expr, $addr1: expr], [$($output:tt)*]) => {
        pub struct $name(u16);
        impl Register for $name {
            fn first_addr(&self) -> u8 {
                $addr0
            }
            fn read(&self) -> Vec<u8> {
                vec![$addr0 | READ_FLAG, 0x00, 0x00]
            }

            fn write(&self) -> Vec<u8> {
                let [a, b] = self.0.to_be_bytes();
                vec![$addr0 & WRITE_FLAG, a, b]
            }
            fn new(values: &[u8]) -> Self {
                if values.len() == 3 {
                     Self(u16::from_be_bytes([values[1], values[2]]))
                } else if values.len() == 2 {
                     Self(u16::from_be_bytes([values[0], values[1]]))
                } else {
                    panic!("Wrong values")
                }
            }
        }
    };
    ($name: ident, u32[$addr0: expr, $addr1: expr, $addr2: expr], [$($output:tt)*]) => {
        pub struct $name(u32);
        impl Register for $name {
            fn first_addr(&self) -> u8 {
                $addr0
            }
            fn read(&self) -> Vec<u8> {
                vec![$addr0 | READ_FLAG, 0x00, 0x00, 0x00]
            }
            fn write(&self) -> Vec<u8> {
                let [a, b, c, _] = self.0.to_be_bytes();
                vec![$addr0 & WRITE_FLAG, a, b, c]
            }
            fn new(values: &[u8]) -> Self {
                if values.len() == 4 {
                     Self(u32::from_be_bytes([values[1], values[2], values[3], 0x00]))
                } else if values.len() == 3 {
                     Self(u32::from_be_bytes([values[0], values[1], values[2], 0x00]))
                } else {
                    panic!("Wrong values")
                }
            }
        }
    };
    ($name: ident, u32[$addr0: expr, $addr1: expr, $addr2: expr, $addr3: expr], [$($output:tt)*]) => {
        pub struct $name(u32);
        impl Register for $name {
            fn first_addr(&self) -> u8 {
                $addr0
            }
            fn read(&self) -> Vec<u8> {
                vec![$addr0 | READ_FLAG, 0x00, 0x00, 0x00, 0x00]
            }
            fn write(&self) -> Vec<u8> {
                let [a, b, c, d] = self.0.to_be_bytes();
                vec![$addr0 & WRITE_FLAG, a, b, c, d]
            }
            fn new(values: &[u8]) -> Self {
                if values.len() == 5 {
                     Self(u32::from_be_bytes([values[1], values[2], values[3], values[4]]))
                } else if values.len() == 3 {
                     Self(u32::from_be_bytes([values[0], values[1], values[2], values[3]]))
                } else {
                    panic!("Wrong values")
                }
            }
        }
    };
}

macro_rules! register_field {
    ($tt:ty[$getter:ident, $setter:ident]: $t:ty[$lsb:expr, $msb: expr]) => {
            pub fn $getter(&self) -> $t {
                let mut mask:$tt = 0;
                for _i in $lsb..=$msb {
                    mask <<= 1;
                    mask |= 0x01
                }
                mask = mask << $lsb;

                ((self.0 & mask) >> $lsb) as $t
            }
            pub fn $setter(&mut self, value: $t) {
                let bit_len = (size_of::<$tt>()*8) as $tt;
                let mask: $tt = !(0 as $tt)
                    << (bit_len - ($msb as $tt) - 1)
                    >> (bit_len - ($msb as $tt) - 1 + ($lsb as $tt))
                    << (($lsb as $tt));
                self.0 &= !mask;
                self.0 |= (value as $tt << ($lsb as $tt)) & mask;
            }
    };
}


create_register!(DeviceType, u8[0], [dt:5]);
impl DeviceType {
    register_field!(u8[dt, set_dt]: u8[0, 4]);
}


create_register!(DeviceVersion, u8[1], [vc:5]);
impl DeviceVersion {
    register_field!(u8[vc, set_vc]: u8[0, 4]);
}


create_register!(DeviceStatus, u8[2], [cps:2, headerr:1, rxffem:1, ffunfl:1, ffovfl:1]);
impl DeviceStatus {
    register_field!(u8[cps, set_cps]: u8[0, 1]);
    register_field!(u8[headerr, set_headerr]: u8[4, 4]);
    register_field!(u8[rxffem, set_rxffem]: u8[5, 5]);
    register_field!(u8[ffunfl, set_ffunfl]: u8[6, 6]);
    register_field!(u8[ffovfl, set_ffovfl]: u8[7, 7]);
}


create_register!(InterruptStatus, u16[3, 4], [icrcerror:1, ipkvalid:1, ipksent:1, iext:1, irxffafull:1, itxffaem:1, itxffafull:1, ifferr:1, ipor:1, ichiprdy:1, ilbd:1, iwut:1, irssi:1, ipreainval:1, ipreaval:1, iswdet:1]);
impl InterruptStatus {
    register_field!(u16[icrcerror, set_icrcerror]: u8[0, 0]);
    register_field!(u16[ipkvalid, set_ipkvalid]: u8[1, 1]);
    register_field!(u16[ipksent, set_ipksent]: u8[2, 2]);
    register_field!(u16[iext, set_iext]: u8[3, 3]);
    register_field!(u16[irxffafull, set_irxffafull]: u8[4, 4]);
    register_field!(u16[itxffaem, set_itxffaem]: u8[5, 5]);
    register_field!(u16[itxffafull, set_itxffafull]: u8[6, 6]);
    register_field!(u16[ifferr, set_ifferr]: u8[7, 7]);
    register_field!(u16[ipor, set_ipor]: u8[8, 8]);
    register_field!(u16[ichiprdy, set_ichiprdy]: u8[9, 9]);
    register_field!(u16[ilbd, set_ilbd]: u8[10, 10]);
    register_field!(u16[iwut, set_iwut]: u8[11, 11]);
    register_field!(u16[irssi, set_irssi]: u8[12, 12]);
    register_field!(u16[ipreainval, set_ipreainval]: u8[13, 13]);
    register_field!(u16[ipreaval, set_ipreaval]: u8[14, 14]);
    register_field!(u16[iswdet, set_iswdet]: u8[15, 15]);
}


create_register!(InterruptEnable, u16[5, 6], [encrcerror:1, enpkvalid:1, enpksent:1, enext:1, enrxffafull:1, entxffaem:1, entxffafull:1, enfferr:1, enpor:1, enchiprdy:1, enlbd:1, enwut:1, enrssi:1, enpreainval:1, enpreaval:1, enswde
t:1]);
impl InterruptEnable {
    register_field!(u16[encrcerror, set_encrcerror]: u8[0, 0]);
    register_field!(u16[enpkvalid, set_enpkvalid]: u8[1, 1]);
    register_field!(u16[enpksent, set_enpksent]: u8[2, 2]);
    register_field!(u16[enext, set_enext]: u8[3, 3]);
    register_field!(u16[enrxffafull, set_enrxffafull]: u8[4, 4]);
    register_field!(u16[entxffaem, set_entxffaem]: u8[5, 5]);
    register_field!(u16[entxffafull, set_entxffafull]: u8[6, 6]);
    register_field!(u16[enfferr, set_enfferr]: u8[7, 7]);
    register_field!(u16[enpor, set_enpor]: u8[8, 8]);
    register_field!(u16[enchiprdy, set_enchiprdy]: u8[9, 9]);
    register_field!(u16[enlbd, set_enlbd]: u8[10, 10]);
    register_field!(u16[enwut, set_enwut]: u8[11, 11]);
    register_field!(u16[enrssi, set_enrssi]: u8[12, 12]);
    register_field!(u16[enpreainval, set_enpreainval]: u8[13, 13]);
    register_field!(u16[enpreaval, set_enpreaval]: u8[14, 14]);
    register_field!(u16[enswdet, set_enswdet]: u8[15, 15]);
}


create_register!(OperatingAndFunctionControl, u16[7, 8], [xton:1, pllon:1, rxon:1, txon:1, x32ksel:1, enwt:1, enlbd:1, swres:1, ffclrtx:1, ffclrrx:1, enldm:1, autotx:1, rxmpk:1, antdiv:3]);
impl OperatingAndFunctionControl {
    register_field!(u16[xton, set_xton]: u8[0, 0]);
    register_field!(u16[pllon, set_pllon]: u8[1, 1]);
    register_field!(u16[rxon, set_rxon]: u8[2, 2]);
    register_field!(u16[txon, set_txon]: u8[3, 3]);
    register_field!(u16[x32ksel, set_x32ksel]: u8[4, 4]);
    register_field!(u16[enwt, set_enwt]: u8[5, 5]);
    register_field!(u16[enlbd, set_enlbd]: u8[6, 6]);
    register_field!(u16[swres, set_swres]: u8[7, 7]);
    register_field!(u16[ffclrtx, set_ffclrtx]: u8[8, 8]);
    register_field!(u16[ffclrrx, set_ffclrrx]: u8[9, 9]);
    register_field!(u16[enldm, set_enldm]: u8[10, 10]);
    register_field!(u16[autotx, set_autotx]: u8[11, 11]);
    register_field!(u16[rxmpk, set_rxmpk]: u8[12, 12]);
    register_field!(u16[antdiv, set_antdiv]: u8[13, 15]);
}


create_register!(CrystalOscillatorLoadCapacitance, u8[9], [xlc:7, xtalshft:1]);
impl CrystalOscillatorLoadCapacitance {
    register_field!(u8[xlc, set_xlc]: u8[0, 6]);
    register_field!(u8[xtalshft, set_xtalshft]: u8[7, 7]);
}


create_register!(MicrocontrollerOutputClock, u8[10], [mclk:3, enlfc:1, clkt:2]);
impl MicrocontrollerOutputClock {
    register_field!(u8[mclk, set_mclk]: u8[0, 2]);
    register_field!(u8[enlfc, set_enlfc]: u8[3, 3]);
    register_field!(u8[clkt, set_clkt]: u8[4, 5]);
}


create_register!(GPIO0Configuration, u8[11], [gpio0:5, pup0:1, gpio0drv:2]);
impl GPIO0Configuration {
    register_field!(u8[gpio0, set_gpio0]: u8[0, 4]);
    register_field!(u8[pup0, set_pup0]: u8[5, 5]);
    register_field!(u8[gpio0drv, set_gpio0drv]: u8[6, 7]);
}


create_register!(GPIO1Configuration, u8[12], [gpio1:5, pup1:1, gpio1drv:2]);
impl GPIO1Configuration {
    register_field!(u8[gpio1, set_gpio1]: u8[0, 4]);
    register_field!(u8[pup1, set_pup1]: u8[5, 5]);
    register_field!(u8[gpio1drv, set_gpio1drv]: u8[6, 7]);
}


create_register!(GPIO2Configuration, u8[13], [gpio2:5, pup2:1, gpio2drv:2]);
impl GPIO2Configuration {
    register_field!(u8[gpio2, set_gpio2]: u8[0, 4]);
    register_field!(u8[pup2, set_pup2]: u8[5, 5]);
    register_field!(u8[gpio2drv, set_gpio2drv]: u8[6, 7]);
}


create_register!(IOPortConfiguration, u8[14], [dio0:1, dio1:1, dio2:1, itsdo:1, extitst:3]);
impl IOPortConfiguration {
    register_field!(u8[dio0, set_dio0]: u8[0, 0]);
    register_field!(u8[dio1, set_dio1]: u8[1, 1]);
    register_field!(u8[dio2, set_dio2]: u8[2, 2]);
    register_field!(u8[itsdo, set_itsdo]: u8[3, 3]);
    register_field!(u8[extitst, set_extitst]: u8[4, 6]);
}


create_register!(ADCConfiguration, u8[15], [adcgain:2, adcref:2, adcsel:3, adcstart:1]);
impl ADCConfiguration {
    register_field!(u8[adcgain, set_adcgain]: u8[0, 1]);
    register_field!(u8[adcref, set_adcref]: u8[2, 3]);
    register_field!(u8[adcsel, set_adcsel]: u8[4, 6]);
    register_field!(u8[adcstart, set_adcstart]: u8[7, 7]);
}


create_register!(ADCSensorAmplifierOffset, u8[16], [adcoffs:4]);
impl ADCSensorAmplifierOffset {
    register_field!(u8[adcoffs, set_adcoffs]: u8[0, 3]);
}


create_register!(ADCValue, u8[17], [adc:8]);
impl ADCValue {
    register_field!(u8[adc, set_adc]: u8[0, 7]);
}


create_register!(TemperatureSensorControl, u8[18], [tstrim:4, entstrim:1, entsoffs:1, tsrange:2]);
impl TemperatureSensorControl {
    register_field!(u8[tstrim, set_tstrim]: u8[0, 3]);
    register_field!(u8[entstrim, set_entstrim]: u8[4, 4]);
    register_field!(u8[entsoffs, set_entsoffs]: u8[5, 5]);
    register_field!(u8[tsrange, set_tsrange]: u8[6, 7]);
}


create_register!(TemperatureValueOffset, u8[19], [tvoffs:8]);
impl TemperatureValueOffset {
    register_field!(u8[tvoffs, set_tvoffs]: u8[0, 7]);
}


create_register!(WakeUpTimerPeriod, u32[20, 21, 22], [wtr:5, wtm:16]);
impl WakeUpTimerPeriod {
    register_field!(u32[wtr, set_wtr]: u8[0, 4]);
    register_field!(u32[wtm, set_wtm]: u16[8, 23]);
}


create_register!(WakeUpTimerValue, u16[23, 24], [wtv:16]);
impl WakeUpTimerValue {
    register_field!(u16[wtv, set_wtv]: u16[0, 15]);
}


create_register!(LowDutyCycleModeDuration, u8[25], [ldc:8]);
impl LowDutyCycleModeDuration {
    register_field!(u8[ldc, set_ldc]: u8[0, 7]);
}


create_register!(LowBatteryDetectorThreshold, u8[26], [lbdt:5]);
impl LowBatteryDetectorThreshold {
    register_field!(u8[lbdt, set_lbdt]: u8[0, 4]);
}


create_register!(BatteryVoltageLevel, u8[27], [vbat:5]);
impl BatteryVoltageLevel {
    register_field!(u8[vbat, set_vbat]: u8[0, 4]);
}


create_register!(IFFilterBandwidth, u8[28], [filset:4, ndec:3, dwn3_bypass:1]);
impl IFFilterBandwidth {
    register_field!(u8[filset, set_filset]: u8[0, 3]);
    register_field!(u8[ndec, set_ndec]: u8[4, 6]);
    register_field!(u8[dwn3_bypass, set_dwn3_bypass]: u8[7, 7]);
}


create_register!(AFCLoopGearshiftOverride, u8[29], [ph0size:1, matap:1, bypass1p5:1, afcgearh:3, enafc:1, afcbd:1]);
impl AFCLoopGearshiftOverride {
    register_field!(u8[ph0size, set_ph0size]: u8[0, 0]);
    register_field!(u8[matap, set_matap]: u8[1, 1]);
    register_field!(u8[bypass1p5, set_bypass1p5]: u8[2, 2]);
    register_field!(u8[afcgearh, set_afcgearh]: u8[3, 5]);
    register_field!(u8[enafc, set_enafc]: u8[6, 6]);
    register_field!(u8[afcbd, set_afcbd]: u8[7, 7]);
}


create_register!(AFCTimingControl, u8[30], [anwait:3, shwait:3, swait_timer:2]);
impl AFCTimingControl {
    register_field!(u8[anwait, set_anwait]: u8[0, 2]);
    register_field!(u8[shwait, set_shwait]: u8[3, 5]);
    register_field!(u8[swait_timer, set_swait_timer]: u8[6, 7]);
}


create_register!(ClockRecoveryGearshiftOverride, u8[31], [crslow:3, crfast:3]);
impl ClockRecoveryGearshiftOverride {
    register_field!(u8[crslow, set_crslow]: u8[0, 2]);
    register_field!(u8[crfast, set_crfast]: u8[3, 5]);
}


create_register!(ClockRecoveryOversampling, u8[32], [rxosr:8]);
impl ClockRecoveryOversampling {
    register_field!(u8[rxosr, set_rxosr]: u8[0, 7]);
}


create_register!(ClockRecoveryOffset, u32[33, 34, 35], [ncoff:20, stallctrl:1, rxosr:3]);
impl ClockRecoveryOffset {
    register_field!(u32[ncoff, set_ncoff]: u32[0, 19]);
    register_field!(u32[stallctrl, set_stallctrl]: u8[20, 20]);
    register_field!(u32[rxosr, set_rxosr]: u8[21, 23]);
}


create_register!(ClockRecoveryTimingLoop, u16[36, 37], [crgain:11, crgain2x:1, rxncocomp:1]);
impl ClockRecoveryTimingLoop {
    register_field!(u16[crgain, set_crgain]: u16[0, 10]);
    register_field!(u16[crgain2x, set_crgain2x]: u8[11, 11]);
    register_field!(u16[rxncocomp, set_rxncocomp]: u8[12, 12]);
}


create_register!(ReceivedSignalStrengthIndicator, u8[38], [rssi:8]);
impl ReceivedSignalStrengthIndicator {
    register_field!(u8[rssi, set_rssi]: u8[0, 7]);
}


create_register!(RSSIThresholdforClearChannelIndicator, u8[39], [rssith:8]);
impl RSSIThresholdforClearChannelIndicator {
    register_field!(u8[rssith, set_rssith]: u8[0, 7]);
}


create_register!(AntennaDiversityRegister, u16[40, 41], [adrssia:8, adrssib:8]);
impl AntennaDiversityRegister {
    register_field!(u16[adrssia, set_adrssia]: u8[0, 7]);
    register_field!(u16[adrssib, set_adrssib]: u8[8, 15]);
}


create_register!(AFCLimiter, u8[42], [afclim:8]);
impl AFCLimiter {
    register_field!(u8[afclim, set_afclim]: u8[0, 7]);
}


create_register!(AFCCorrectionRead, u8[43], [afc_corr:8]);
impl AFCCorrectionRead {
    register_field!(u8[afc_corr, set_afc_corr]: u8[0, 7]);
}


create_register!(OOKCounterValue, u16[44, 45], [ookcnt:11, madeten:1, peakdeten:1, ookfrzen:1, afc_corr:2]);
impl OOKCounterValue {
    register_field!(u16[ookcnt, set_ookcnt]: u16[0, 10]);
    register_field!(u16[madeten, set_madeten]: u8[11, 11]);
    register_field!(u16[peakdeten, set_peakdeten]: u8[12, 12]);
    register_field!(u16[ookfrzen, set_ookfrzen]: u8[13, 13]);
    register_field!(u16[afc_corr, set_afc_corr]: u8[14, 15]);
}


create_register!(SlicerPeakHold, u8[46], [decay:4, attack:3]);
impl SlicerPeakHold {
    register_field!(u8[decay, set_decay]: u8[0, 3]);
    register_field!(u8[attack, set_attack]: u8[4, 6]);
}


create_register!(DataAccessControl, u8[48], [crc:2, encrc:1, enpactx:1, skip2ph:1, crcdonly:1, lsbfrst:1, enpacrx:1]);
impl DataAccessControl {
    register_field!(u8[crc, set_crc]: u8[0, 1]);
    register_field!(u8[encrc, set_encrc]: u8[2, 2]);
    register_field!(u8[enpactx, set_enpactx]: u8[3, 3]);
    register_field!(u8[skip2ph, set_skip2ph]: u8[4, 4]);
    register_field!(u8[crcdonly, set_crcdonly]: u8[5, 5]);
    register_field!(u8[lsbfrst, set_lsbfrst]: u8[6, 6]);
    register_field!(u8[enpacrx, set_enpacrx]: u8[7, 7]);
}


create_register!(EzMACstatus, u8[49], [pksent:1, pktx:1, crcerror:1, pkvalid:1, pkrx:1, pksrch:1, rxcrc1:1]);
impl EzMACstatus {
    register_field!(u8[pksent, set_pksent]: u8[0, 0]);
    register_field!(u8[pktx, set_pktx]: u8[1, 1]);
    register_field!(u8[crcerror, set_crcerror]: u8[2, 2]);
    register_field!(u8[pkvalid, set_pkvalid]: u8[3, 3]);
    register_field!(u8[pkrx, set_pkrx]: u8[4, 4]);
    register_field!(u8[pksrch, set_pksrch]: u8[5, 5]);
    register_field!(u8[rxcrc1, set_rxcrc1]: u8[6, 6]);
}


create_register!(HeaderControl, u16[50, 51], [hdch:4, bcen:4, prealen:1, synclen:2, fixpklen:1, hdlen:3, skipsyn:1]);
impl HeaderControl {
    register_field!(u16[hdch, set_hdch]: u8[0, 3]);
    register_field!(u16[bcen, set_bcen]: u8[4, 7]);
    register_field!(u16[prealen, set_prealen]: u8[8, 8]);
    register_field!(u16[synclen, set_synclen]: u8[9, 10]);
    register_field!(u16[fixpklen, set_fixpklen]: u8[11, 11]);
    register_field!(u16[hdlen, set_hdlen]: u8[12, 14]);
    register_field!(u16[skipsyn, set_skipsyn]: u8[15, 15]);
}


create_register!(PreambleLength, u8[52], [prealen:8]);
impl PreambleLength {
    register_field!(u8[prealen, set_prealen]: u8[0, 7]);
}


create_register!(PreambleDetectionControl, u8[53], [rssi_off:3, preath:5]);
impl PreambleDetectionControl {
    register_field!(u8[rssi_off, set_rssi_off]: u8[0, 2]);
    register_field!(u8[preath, set_preath]: u8[3, 7]);
}


create_register!(SyncWord, u32[54, 55, 56, 57], [sync:32]);
impl SyncWord {
    register_field!(u32[sync, set_sync]: u32[0, 31]);
}


create_register!(TransmitHeader, u32[58, 59, 60, 61], [txhd:32]);
impl TransmitHeader {
    register_field!(u32[txhd, set_txhd]: u32[0, 31]);
}


create_register!(TransmitPacketLength, u8[62], [pklen:8]);
impl TransmitPacketLength {
    register_field!(u8[pklen, set_pklen]: u8[0, 7]);
}


create_register!(CheckHeader, u32[63, 64, 65, 66], [chhd:32]);
impl CheckHeader {
    register_field!(u32[chhd, set_chhd]: u32[0, 31]);
}


create_register!(HeaderEnable, u32[67, 68, 69, 70], [hden:32]);
impl HeaderEnable {
    register_field!(u32[hden, set_hden]: u32[0, 31]);
}


create_register!(ReceivedHeader, u32[71, 72, 73, 74], [rxhd:32]);
impl ReceivedHeader {
    register_field!(u32[rxhd, set_rxhd]: u32[0, 31]);
}


create_register!(ReceivedPacketLength, u8[75], [rxplen:8]);
impl ReceivedPacketLength {
    register_field!(u8[rxplen, set_rxplen]: u8[0, 7]);
}


create_register!(ADC8Control, u8[79], [adc8:6]);
impl ADC8Control {
    register_field!(u8[adc8, set_adc8]: u8[0, 5]);
}


create_register!(ChannelFilterCoefficientAddress, u8[96], [chfiladd:4, inv_pre_th:4]);
impl ChannelFilterCoefficientAddress {
    register_field!(u8[chfiladd, set_chfiladd]: u8[0, 3]);
    register_field!(u8[inv_pre_th, set_inv_pre_th]: u8[4, 7]);
}


create_register!(CrystalOscillatorControlTest, u8[98], [enbuf:1, bufovr:1, enamp2x:1, enbias2x:1, clkhyst:1, pwst:3]);
impl CrystalOscillatorControlTest {
    register_field!(u8[enbuf, set_enbuf]: u8[0, 0]);
    register_field!(u8[bufovr, set_bufovr]: u8[1, 1]);
    register_field!(u8[enamp2x, set_enamp2x]: u8[2, 2]);
    register_field!(u8[enbias2x, set_enbias2x]: u8[3, 3]);
    register_field!(u8[clkhyst, set_clkhyst]: u8[4, 4]);
    register_field!(u8[pwst, set_pwst]: u8[5, 7]);
}


create_register!(AGCOverride, u8[105], [pga0:1, pga1:1, pga2:1, pga3:1, lnagain:1, agcen:1, sgi:1]);
impl AGCOverride {
    register_field!(u8[pga0, set_pga0]: u8[0, 0]);
    register_field!(u8[pga1, set_pga1]: u8[1, 1]);
    register_field!(u8[pga2, set_pga2]: u8[2, 2]);
    register_field!(u8[pga3, set_pga3]: u8[3, 3]);
    register_field!(u8[lnagain, set_lnagain]: u8[4, 4]);
    register_field!(u8[agcen, set_agcen]: u8[5, 5]);
    register_field!(u8[sgi, set_sgi]: u8[6, 6]);
}


create_register!(TXPower, u8[109], [txpow:3, ina_sw:1, papeaklvl:2, papeaken:1, papeakval:1]);
impl TXPower {
    register_field!(u8[txpow, set_txpow]: u8[0, 2]);
    register_field!(u8[ina_sw, set_ina_sw]: u8[3, 3]);
    register_field!(u8[papeaklvl, set_papeaklvl]: u8[4, 5]);
    register_field!(u8[papeaken, set_papeaken]: u8[6, 6]);
    register_field!(u8[papeakval, set_papeakval]: u8[7, 7]);
}


create_register!(TXDataRate, u16[110, 111], [txdr:16]);
impl TXDataRate {
    register_field!(u16[txdr, set_txdr]: u16[0, 15]);
}


create_register!(ModulationModeControl, u16[112, 113], [enwhite:1, enmanch:1, enmaninv:1, manppol:1, enphpwdn:1, txdtrtscale:1, modtyp:2, fd:1, eninv:1, dtmod:2, trclk:2]);
impl ModulationModeControl {
    register_field!(u16[enwhite, set_enwhite]: u8[0, 0]);
    register_field!(u16[enmanch, set_enmanch]: u8[1, 1]);
    register_field!(u16[enmaninv, set_enmaninv]: u8[2, 2]);
    register_field!(u16[manppol, set_manppol]: u8[3, 3]);
    register_field!(u16[enphpwdn, set_enphpwdn]: u8[4, 4]);
    register_field!(u16[txdtrtscale, set_txdtrtscale]: u8[5, 5]);
    register_field!(u16[modtyp, set_modtyp]: u8[8, 9]);
    register_field!(u16[fd, set_fd]: u8[10, 10]);
    register_field!(u16[eninv, set_eninv]: u8[11, 11]);
    register_field!(u16[dtmod, set_dtmod]: u8[12, 13]);
    register_field!(u16[trclk, set_trclk]: u8[14, 15]);
}


create_register!(FrequencyDeviation, u8[114], [fd:8]);
impl FrequencyDeviation {
    register_field!(u8[fd, set_fd]: u8[0, 7]);
}


create_register!(FrequencyOffset, u16[115, 116], [fo:10]);
impl FrequencyOffset {
    register_field!(u16[fo, set_fo]: u16[0, 9]);
}


create_register!(FrequencyBandSelect, u8[117], [fb:5, hbsel:1, sbsel:1]);
impl FrequencyBandSelect {
    register_field!(u8[fb, set_fb]: u8[0, 4]);
    register_field!(u8[hbsel, set_hbsel]: u8[5, 5]);
    register_field!(u8[sbsel, set_sbsel]: u8[6, 6]);
}


create_register!(NominalCarrierFrequency, u16[118, 119], [fc:16]);
impl NominalCarrierFrequency {
    register_field!(u16[fc, set_fc]: u16[0, 15]);
}


create_register!(FrequencyHoppingChannelSelect, u8[121], [fhch:8]);
impl FrequencyHoppingChannelSelect {
    register_field!(u8[fhch, set_fhch]: u8[0, 7]);
}


create_register!(FrequencyHoppingStepSize, u8[122], [fhs:8]);
impl FrequencyHoppingStepSize {
    register_field!(u8[fhs, set_fhs]: u8[0, 7]);
}


create_register!(TXFIFOControl, u16[124, 125], [txafthr:6, txaethr:6]);
impl TXFIFOControl {
    register_field!(u16[txafthr, set_txafthr]: u8[0, 5]);
    register_field!(u16[txaethr, set_txaethr]: u8[8, 13]);
}


create_register!(RXFIFOControl, u8[126], [rxafthr:6]);
impl RXFIFOControl {
    register_field!(u8[rxafthr, set_rxafthr]: u8[0, 5]);
}


create_register!(FIFOAccess, u8[127], [fifod:8]);
impl FIFOAccess {
    register_field!(u8[fifod, set_fifod]: u8[0, 7]);
}




#[cfg(test)]
mod tests{
    use super::*;

    #[test]
    fn register_test() {
        let p = FIFOAccess(0xFF);

        assert_eq!(p.fifod(), 0xFF);

        let mut e = TXFIFOControl(0x0);
        println!("{:08b} {:08b} {:016b}", e.txafthr(), e.txaethr(), e.0);
        e.set_txaethr(0x01);
        e.set_txafthr(0x01);
        println!("{:08b} {:08b} {:016b}", e.txafthr(), e.txaethr(), e.0);

        print!("{:08b} [", e.first_addr());
        for i in e.read().iter() {
            print!("{:08b}, ", i)
        }
        print!("]\n");

        print!("{:08b} [", e.first_addr());
        for i in e.write().iter() {
            print!("{:08b}, ", i)
        }
        print!("]\n");
    }
}

