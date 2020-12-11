#![cfg_attr(not(feature = "std"), no_std)]
#![allow(clippy::manual_range_contains)]
#![feature(step_trait)]

use bounded_integer::bounded_integer;
use embedded_hal::blocking::i2c::{Read, Write};

pub const SEGMENTS_SIZE: usize  = 16;
pub const COMMONS_SIZE: usize   = 8;

pub mod command {
    pub const DIMMING: u8       = 0b1110_0000;
    pub const DISPLAY: u8       = 0b1000_0000;
    pub const OSCILLATOR: u8    = 0b0010_0000;
}

bounded_integer! {
/// Holds the range of values a dimmer can have.
/// Z0 being the lowest value and P15 being the largest.
#[repr(u8)]
pub enum Dimming { 0..16 }
}

// Holds display states
#[derive(Copy, Clone, Debug, Hash)]
pub enum Display {
    Off     = 0b0000_0000,
    On      = 0b0000_0001,
    TwoHz   = 0b0000_0011,
    OneHz   = 0b0000_0101,
    HalfHz  = 0b0000_0111,
}

/// Holds oscillators 2 states
#[derive(Copy, Clone, Debug, Hash)]
pub enum Oscillator {
    On  = 1,
    Off = 0,
}

type Result<Error> = core::result::Result<(), Error>;

/// Main driver. Holds the state and buffer.
#[derive(Copy, Clone, Debug, Hash)]
pub struct HT16K33<I2C> {
    i2c:     I2C,
    address: u8,

    pub buffer: [u8; SEGMENTS_SIZE],

    dimming_level:    Dimming,
    display_state:    Display,
    oscillator_state: Oscillator,
}

impl<I2C, E> HT16K33<I2C>
where
    I2C: Read<Error = E> + Write<Error = E>
{
    /// Creates a new instance.
    /// Should be followed by init().
    pub fn new(i2c: I2C, address: u8) -> Self {
        HT16K33 {
            address,
            i2c,

            buffer: [0; SEGMENTS_SIZE],

            dimming_level:    Dimming::P15,
            display_state:    Display::Off,
            oscillator_state: Oscillator::Off,
        }
    }

    /// Turns on oscillator, display, turns dimming to max, 
    /// clears and writes buffer.
    /// 
    /// These are just five linear steps, you can easily start your driver
    /// diffrently.
    pub fn init(&mut self) -> Result<E> {
        self.write_oscillator(Oscillator::On)?;
        self.write_display(Display::On)?;
        self.write_dimming(Dimming::P15)?;

        self.clear_buffer();
        self.write_buffer()
    }

    /// Returns i2c interface.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// This method allows for almost direct access to the i2c field, 
    /// with only the address set. 
    /// 
    /// Use with caution! See the sourcecode of this library of how to use it.
    pub fn write(&mut self, slice: &[u8]) -> Result<E> {
        self.i2c.write(self.address, slice)
    }

    /// Clears the buffer without writing to controller.
    /// Should perhaps be followed by `write_buffer()`
    pub fn clear_buffer(&mut self) {
        self.buffer = [0; SEGMENTS_SIZE];
    }

    /// Reads in buffer from controller.
    pub fn read_buffer(&mut self, buffer: &mut [u8; SEGMENTS_SIZE]) -> Result<E> {
        self.i2c.read(self.address, buffer)
    }

    /// Writes internal buffer to controller.
    pub fn write_buffer(&mut self) -> Result<E> {
        let mut write_buffer = [0u8; SEGMENTS_SIZE + 1];
        write_buffer[0] = 0;
        write_buffer[1..].clone_from_slice(&self.buffer[..]);
        self.write(&write_buffer)
    }

    /// Return local dimming level. Might not reflect controller.
    pub fn dimming(&self) -> Dimming {
        self.dimming_level
    }

    /// Writes a new dimming level to controller.
    pub fn write_dimming(&mut self, dimming: Dimming) -> Result<E> {
        self.dimming_level = dimming;
        self.write(&[command::DIMMING | dimming as u8])
    }

    /// Returns local display state. Might not reflect controller.
    pub fn display(&self) -> Display {
        self.display_state
    }

    /// Writes a new display state to controller.
    pub fn write_display(&mut self, display: Display) -> Result<E> {
        self.display_state = display;
        self.write(&[command::DISPLAY | display as u8])
    }

    /// Returns local oscillator state. Might not reflect controller.
    pub fn oscillator(&self) -> Oscillator {
        self.oscillator_state
    }

    /// Writes a new oscillator state to controller.
    pub fn write_oscillator(&mut self, oscillator: Oscillator) -> Result<E> {
        self.oscillator_state = oscillator;
        self.write(&[command::OSCILLATOR | oscillator as u8])
    }
}
