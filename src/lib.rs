#![no_std]

#[macro_use]
extern crate alloc;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::Transfer;

use crate::device::Device;

mod device;
mod registers;
mod errors;


pub fn new<SPI: 'static + Transfer<u8>, nSEL: OutputPin, SDN: OutputPin>(spi: &'static mut SPI, nSEL: nSEL, sdn: SDN,) -> Device<'static, SPI, nSEL, SDN> {
    Device::new(spi, nSEL, sdn)
}


#[cfg(test)]
mod tests {}