use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

use crate::registers::Register;
use crate::idle::IdleState;

pub trait Device {
    fn shutdown(&mut self) -> Result<(), &str>;
    fn powerup(&mut self) -> Result<(), &str>;
    fn raw_transfer<'b>(&mut self, r: &'b mut[u8]) -> Result<&'b[u8], &str>;
    fn read<R: Register>(&mut self, r: &R) -> Result<R, &str>;
    fn write<R: Register>(&mut self, r: &R) -> Result<(), &str>;
}
