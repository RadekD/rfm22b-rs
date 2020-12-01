use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

use crate::device::State::Shutdown;
use crate::errors;
use crate::registers::Register;

pub enum Mode {
    Standby,
    Sleep,
    Sensor,
    Ready,
    Tune,
}

pub enum State {
    Shutdown,
    Idle(Mode),
    Tx,
    Rx,
}


pub struct Device<'a, SPI, nSEL, SDN> where SPI: Transfer<u8>, nSEL: OutputPin, SDN: OutputPin {
    spi: &'a mut SPI,
    nSEL: nSEL,
    sdn: SDN,

    state: State,
}

impl<'a, SPI, nSEL, SDN> Device<'a, SPI, nSEL, SDN> where SPI: Transfer<u8>, nSEL: OutputPin, SDN: OutputPin {
    pub fn new(spi: &'a mut SPI, nSEL: nSEL, SDN: SDN) -> Self {
        Self {
            spi: spi,
            nSEL,
            sdn: SDN,
            state: Shutdown,
        }
    }
    pub fn set_state(&mut self, state: State) -> Result<(), errors::Error<(), SDN::Error>> {
        match state {
            State::Shutdown => {
                match self.sdn.set_high() {
                    Ok(()) => {}
                    Err(err) => return Err(errors::Error::Pin(err))
                }

                self.state = state
            }
            State::Idle(idle) => {
                match self.sdn.set_low() {
                    Ok(()) => {}
                    Err(err) => return Err(errors::Error::Pin(err))
                }
                match idle {
                    Mode::Standby => {}
                    Mode::Sleep => {}
                    Mode::Sensor => {}
                    Mode::Ready => {}
                    Mode::Tune => {}
                }
            }
            State::Tx => {}
            State::Rx => {}
        }

        Ok(())
    }

    fn read<R: Register>(&mut self, r: R) -> Result<R, &str> {
        match self.nSEL.set_low() {
            Ok(_) => {}
            Err(_) => {
                return Err("internal")
            }
        };
        let mut to_read = r.read();
        let rslt = self.spi.transfer(to_read.as_mut_slice());
        match self.nSEL.set_high() {
            Ok(_) => {}
            Err(_) => {
                return Err("internal")
            }
        };
        return match rslt {
            Ok(data) => {
                Ok(R::new(data))
            }
            Err(_) => {
                Err("Internal error")
            }
        }
    }
    fn write<R: Register>(&mut self, reg: u8, r: R) -> Result<(), &str> {
        match self.nSEL.set_low() {
            Ok(_) => {}
            Err(_) => {
                return Err("internal")
            }
        };
        let mut to_write = r.write();
        let rslt = self.spi.transfer(to_write.as_mut_slice());
        match self.nSEL.set_high() {
            Ok(_) => {}
            Err(_) => {
                return Err("internal")
            }
        };
        return match rslt {
            Ok(_) => { Ok(()) }
            Err(_) => { Err("internal error") }
        }
    }
}