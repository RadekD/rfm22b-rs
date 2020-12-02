use embedded_hal::timer::CountDown;

use crate::device::Device;
use crate::idle::IdleState;
use crate::registers::*;
use crate::rx::RxState;

pub struct TxState<D: Device> {
    device: D
}

impl<D: Device> TxState<D> {
    pub(crate) fn new(mut d: D) -> Self {
        d.powerup().unwrap();

        Self {
            device: d,
        }
    }
    pub fn idle(self) -> IdleState<D> {
        IdleState::new(self.device)
    }
    pub fn rx(self) -> RxState<D> {
        RxState::new(self.device)
    }

    fn clear_fifo(&mut self) -> Result<(), &str> {
        let mut r = OperatingAndFunctionControl::default();
        r.set_ffclrtx(1);
        match self.device.write(&r) {
            Ok(_) => {
                let mut r = OperatingAndFunctionControl::default();
                r.set_ffclrtx(0);
                self.device.write(&r)?;
                Ok(())
            }
            Err(_) => Err("internal")
        }
    }
    fn set_transmit_packet_length(&mut self, len: u8) -> Result<(), &str> {
        self.device.write(&TransmitPacketLength::new(&[len]))
    }
    fn enable_tx(&mut self) -> Result<(), &str> {
        let mut r = OperatingAndFunctionControl::default();
        r.set_txon(1);
        r.set_xton(1);

        self.device.write(&r)
    }

    pub fn send<T: CountDown>(&mut self, to_send: &mut [u8; 8], timeout: &mut T) -> Result<([u8; 8], u8), &str> {
        self.clear_fifo().unwrap();
        self.set_transmit_packet_length(64).unwrap();

        let fifo = FIFOAccess::new(to_send);
        match self.device.write(&fifo) {
            Ok(_) => {}
            Err(_) => { return Err("internal"); }
        }

        loop {
            match timeout.wait() {
                Ok(_) => {
                    return Err("timeout");
                }
                _ => {}
            }

            let r = self.device.read(&OperatingAndFunctionControl::default()).unwrap();
            if r.txon() == 0x00 {
                return Ok((fifo.fifod().to_be_bytes(), 64));
            }
        }
    }
}