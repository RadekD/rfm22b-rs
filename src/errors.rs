#![allow(unused)]
pub enum Error<S, P> {
    Internal,
    DeviceTurnedOff,
    SPI(S),
    Pin(P),
}