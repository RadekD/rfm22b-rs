use crate::device::Device;
use crate::registers::{FrequencyBandSelect, NominalCarrierFrequency};
use crate::rx::RxState;
use crate::tx::TxState;

use rtt_target::{rprintln};

pub enum Mode {
    Standby,
    Sleep,
    Sensor,
    Ready,
    Tune,
}

pub struct IdleState<D: Device> {
    device: D,
    mode: Mode,
}

const K_UPPER_FREQUENCY: u32 = 960_000_000;
const K_LOWER_FREQUENCY: u32 = 240_000_000;
const K_HIGH_BAND: u32 = 480_000_000;

impl<D: Device> IdleState<D> {
    pub(crate) fn new(mut d: D) -> Self {
        d.powerup().unwrap();

        Self {
            device: d,
            mode: Mode::Ready,
        }
    }
    pub fn rx(self) -> RxState<D> {
        RxState::new(self.device)
    }
    pub fn tx(self) -> TxState<D> {
        TxState::new(self.device)
    }

    pub fn get_carrier_frequency(&mut self) -> Result<u32, &str> {
        let freq_band = self.device.read(&FrequencyBandSelect::default()).unwrap();
        let nfc = self.device.read(&NominalCarrierFrequency::default()).unwrap();

        let fb = freq_band.fb() as u32;
        let hbsel = freq_band.hbsel() as u32;
        let fc = nfc.fc() as u32;
        let megahz:u32 = 1_000_000;

        let freq = megahz * ((hbsel) + 1) * ((fb) + 24 + (fc / 64000));
        Ok(freq)
    }
    pub fn set_carrier_frequency(&mut self, freq: u32) -> Result<(), &str> {
        if freq < K_LOWER_FREQUENCY || freq > K_UPPER_FREQUENCY {
            return Err("Freq out of range");
        }
        let hbsel = if freq >= K_HIGH_BAND { 1 } else { 0 };
        let fb = (((freq / 10_000_000) / (hbsel + 1)) - 24);

        let xx = 10_000_000 * (hbsel + 1);
        let yy = freq / xx;
        let fc = (yy - fb - 24) * 64000;

        rprintln!("FREQ: {} {} {} HBSEL: {} FB: {} FC: {}",xx, yy, freq, hbsel, fb, fc);


        let mut fbs = FrequencyBandSelect::default();
        fbs.set_sbsel(1);
        fbs.set_hbsel(hbsel as u8);
        fbs.set_fb(fb as u8);

        let mut ncf = NominalCarrierFrequency::default();
        ncf.set_fc(fc as u16);


        match self.device.write(&fbs) {
            Ok(_) => {}
            Err(_) => { return Err("internal"); }
        }
        match self.device.write(&ncf) {
            Ok(_) => {}
            Err(_) => { return Err("internal"); }
        }


        Ok(())
    }
}
