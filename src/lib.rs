#![no_std]

#[macro_use]
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

use crate::device::Device;
use crate::idle::IdleState;
use crate::registers::Register;

pub mod device;
pub mod registers;
pub mod errors;
pub mod idle;
pub mod rx;
pub mod tx;


pub struct RF22B<'a, SPI, nSEL, SDN> where SPI: Transfer<u8>, nSEL: OutputPin, SDN: OutputPin {
    spi: &'a mut SPI,
    nSEL: nSEL,
    pub(crate) sdn: SDN,
}

impl<'a, SPI, nSEL, SDN> RF22B<'a, SPI, nSEL, SDN> where SPI: Transfer<u8>, nSEL: OutputPin, SDN: OutputPin {
    pub fn new(spi: &'a mut SPI, nSEL: nSEL, sdn: SDN) -> IdleState<Self> {
        let device = Self {
            spi,
            nSEL,
            sdn,
        };

        IdleState::new(device)
    }
}

impl<'a, SPI, nSEL, SDN> Device for RF22B<'a, SPI, nSEL, SDN> where SPI: Transfer<u8>, nSEL: OutputPin, SDN: OutputPin {
    fn shutdown(&mut self) -> Result<(), &str> {
        self.sdn.set_high().map_err(|_| "internal error")
    }
    fn powerup(&mut self) -> Result<(), &str> {
        self.sdn.set_high().map_err(|_| "internal error")
    }
    fn raw_transfer<'b>(&mut self, r: &'b mut [u8]) -> Result<&'b [u8], &str> {
        match self.nSEL.set_low() {
            Ok(_) => {}
            Err(_) => {
                return Err("internal");
            }
        };
        let result = self.spi.transfer(r);
        match self.nSEL.set_high() {
            Ok(_) => {}
            Err(_) => {
                return Err("internal");
            }
        };
        match result {
            Ok(data) => Ok(data),
            Err(_) => Err("internal"),
        }
    }

    fn read<R: Register>(&mut self, r: &R) -> Result<R, &str> {
        let mut buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        match self.raw_transfer(&mut r.read(&mut buf)) {
            Ok(data) => Ok(R::new(data)),
            Err(e) => Err(e),
        }
    }

    fn write<R: Register>(&mut self, r: &R) -> Result<(), &str> {
        let mut buf = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        match self.spi.transfer(&mut r.write(&mut buf)) {
            Ok(_) => Ok(()),
            Err(_) => Err("internal"),
        }
    }
}

#[cfg(test)]
mod tests {}