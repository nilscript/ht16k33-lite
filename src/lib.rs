#![cfg_attr(not(feature = "std"), no_std)]
#![allow(clippy::manual_range_contains)]

use bounded_integer::bounded_integer;
use embedded_hal as hal;
use hal::blocking::i2c::{Write, WriteRead};

pub mod command {
    pub const DIMMING: u8 =     0b1110_0000;
    pub const DISPLAY: u8 =     0b1000_0000;
    pub const OSCILLATOR: u8 =  0b0010_0000;
}

bounded_integer! {
    #[repr(u8)]
    pub enum Dimming { 0..16 }
}

#[derive(Copy, Clone)]
pub enum Display {
    Off =       0b0000_0000,
    On =        0b0000_0001,
    HalfHz =    0b0000_0111,
    OneHz =     0b0000_0101,
    TwoHz =     0b0000_0011,
}

#[derive(Copy, Clone)]
pub enum Oscillator {
    On = 1,
    Off = 0,
}

type Result<Error> = core::result::Result<(), Error>;

pub struct HT16K33<I2C> {
    i2c: I2C,
    address: u8,

    pub buffer: [u8; 16],

    dimming_level: Dimming,
    display_state: Display,
    oscillator_state: Oscillator,
}

impl<I2C, E> HT16K33<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        HT16K33 {
            address,
            i2c,
            buffer: [0; 16],
            dimming_level: Dimming::P15,
            display_state: Display::Off,
            oscillator_state: Oscillator::Off,
        }
    }

    pub fn init(&mut self) -> Result<E> {
        self.write_oscillator(Oscillator::On)?;
        self.write_display(Display::On)?;
        self.write_dimming(Dimming::P15)?;

        self.clear_buffer();
        self.write_buffer()
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }

    pub fn clear_buffer(&mut self) {
        self.buffer = [0; 16];
    }

    pub fn read_buffer(&mut self) -> Result<E> {
        self.i2c.write_read(self.address, &[0], &mut self.buffer)
    }

    pub fn write_buffer(&mut self) -> Result<E> {
        let mut write_buffer = [0u8; 16 + 1];
        write_buffer[0] = 0;
        write_buffer[1..(16 + 1)].clone_from_slice(&self.buffer[..]);
        self.i2c.write(self.address, &write_buffer)
    }

    pub fn dimming(&self) -> Dimming {
        self.dimming_level
    }

    pub fn write_dimming(&mut self, dimming: Dimming) -> Result<E> {
        self.dimming_level = dimming;
        self.i2c.write(self.address, &[command::DIMMING | dimming as u8])
    }

    pub fn display(&self) -> Display {
        self.display_state
    }

    pub fn write_display(&mut self, display: Display) -> Result<E> {
        self.display_state = display;
        self.i2c.write(self.address, &[command::DISPLAY | display as u8])
    }

    pub fn oscillator(&self) -> Oscillator {
        self.oscillator_state
    }

    pub fn write_oscillator(&mut self, oscillator: Oscillator) -> Result<E> {
        self.oscillator_state = oscillator;
        self.i2c.write(self.address, &[command::OSCILLATOR | oscillator as u8])
    }
}
