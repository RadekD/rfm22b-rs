use embedded_hal::timer::CountDown;

use crate::device::Device;
use crate::idle::IdleState;
use crate::registers::*;
use crate::tx::TxState;

pub struct RxState<D: Device> {
    device: D,
}

impl<D: Device> RxState<D> {
    pub(crate) fn new(mut d: D) -> Self {
        d.powerup().unwrap();

        Self {
            device: d,
        }
    }
    pub fn idle(self) -> IdleState<D> {
        IdleState::new(self.device)
    }
    pub fn tx(self) -> TxState<D> {
        TxState::new(self.device)
    }

    fn clear_fifo(&mut self) -> Result<(), &str> {
        let mut r = OperatingAndFunctionControl::default();
        r.set_ffclrrx(1);
        match self.device.write(&r) {
            Ok(_) => {
                let mut r = OperatingAndFunctionControl::default();
                r.set_ffclrrx(0);
                self.device.write(&r)?;
                Ok(())
            }
            Err(_) => Err("internal")
        }
    }
    fn enable_rx(&mut self) -> Result<(), &str> {
        let mut r = OperatingAndFunctionControl::default();
        r.set_rxon(1);
        r.set_xton(1);

        self.device.write(&r)
    }

    fn packet_length(&mut self) -> Result<u8, &str> {
        match self.device.read(&ReceivedPacketLength::default()) {
            Ok(r) => Ok(r.rxplen()),
            Err(e) => Err("internal")
        }
    }

    fn receive<C: CountDown>(&mut self, timeout: &mut C) -> Result<([u8; 8], u8), &str> {
        self.clear_fifo().unwrap();
        self.enable_rx().unwrap();

        loop {
            match timeout.wait() {
                Ok(_) => { return Err("timeout") }
                _ => {}
            }
            let control = self.device.read(&OperatingAndFunctionControl::default()).unwrap();
            if control.rxon() == 0x00 {
                break;
            }
        }
        let len = self.packet_length().unwrap();

        let fifo = self.device.read(&FIFOAccess::default());
        Ok((fifo.unwrap().fifod().to_be_bytes(), len))
    }
}