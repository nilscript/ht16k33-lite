#![allow(clippy::manual_range_contains)]
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(step_trait)]

//! 

use embedded_hal::blocking::i2c::{Read, Write};

pub const SEGMENTS_SIZE: usize  = 16;
pub const COMMONS_SIZE: usize   = 8;

/// Contains all commands for each subsystem.
pub mod commands {
    pub const DISPLAY_DATA_ADDRESS_POINTER: u8 = 0b0000_0000; // R/W
    pub const SYSTEM_SETUP:                 u8 = 0b0010_0000; // Write only
    pub const KEY_DATA_ADDRESS_POINTER:     u8 = 0b0100_0000; // Read only
    pub const INT_FLAG_ADDRESS_POINTER:     u8 = 0b0110_0000; // Read only
    pub const DISPLAY_SETUP:                u8 = 0b1000_0000; // Write only
    pub const ROWINT_SET:                   u8 = 0b1010_0000; // Write only
    pub const DIMMING_SET:                  u8 = 0b1110_0000; // Write only
    pub const TEST_MODE_HOLTEK_ONLY:        u8 = 0b1101_1001; // Write only
}

/// Contains all legal states for each subsystem.
pub mod states {
    use bounded_integer::bounded_integer;

    /// System Setup Register States.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum System {
        /// Oscillator off
        StandBy = 0,
        /// Oscillator on
        Normal  = 1,
    }

    /// ROW/INT Set Register States.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum RowInt {
        /// Row driver output
        Row     = 0b0000_0000,
        /// Int output, active low
        IntLow  = 0b0000_0001,
        /// Int output, active high
        IntHigh = 0b0000_0011,
    }

    /// Display Setup Register States.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum Display {
        /// Display off
        Off     = 0b0000_0000,
        /// Display on
        On      = 0b0000_0001,
        /// Display on + Blinking frequency = 2hz
        TwoHz   = 0b0000_0011,
        /// Display on + Blinking frequency = 1hz
        OneHz   = 0b0000_0101,
        /// Display on + Blinking frequency = 0.5hz
        HalfHz  = 0b0000_0111,
    }

    bounded_integer! {
    /// Digital Dimming Data Input Pulse Width Duties.
    /// 
    /// Ranges go from Z, P1..=P15.
    #[repr(u8)]
    pub enum Pulse { 0..=15 }
    }
}

pub mod addresses {
    use bounded_integer::bounded_integer;

    bounded_integer! {
    /// Display Data Address Pointer.
    pub enum DDataAddress { 0..=15 }
    }

    bounded_integer! {
    /// Key Data Address Pointers.
    pub enum KeyData { 0..6 }
    }
}

use commands::*;
use states::*;
use addresses::*;
pub type Result<Error> = core::result::Result<(), Error>;

/// Driver for the HT16K33 RAM Mapping 16*8 LED controller Driver with key scan.
/// 
/// Has an internal state which it tries to reflect
/// onto the connected controller.
/// 
/// Internal buffer is exposed and should be read and edited that way.
/// Write buffer with 
#[derive(Copy, Clone, Debug, Hash)]
pub struct HT16K33<I2C> {
    i2c:      I2C,
    address:  u8,
    dbuf:     [u8; SEGMENTS_SIZE + 1],
    system:   System,
    display:  Display,
    rowint:   RowInt,
    dimming:  Pulse,
}

impl<I2C, E> HT16K33<I2C>
where
    I2C: Read<Error = E> + Write<Error = E>
{
    /// Creates a new instance.
    /// Should be followed by power_on().
    pub fn new(i2c: I2C, address: u8) -> Self {
        let mut dbuf = [0; SEGMENTS_SIZE + 1];
        dbuf[0] = DISPLAY_DATA_ADDRESS_POINTER;

        HT16K33 {
            i2c,
            address,
            dbuf,
            system:   System::StandBy,
            display:  Display::Off,
            rowint:   RowInt::Row,
            dimming:  Pulse::MAX,
        }
    }

    /// Turns system to normal mode, set's Row/Int to Row, turns on display, 
    /// turns dimming to max, and writes what is in display buffer.
    pub fn power_on(&mut self) -> Result<E> {
        self.write_system(System::Normal)?;
        self.write_rowint(RowInt::Row)?;
        self.write_display(Display::On)?;
        self.write_dimming(Pulse::MAX)?;
        self.clear_dbuf();
        self.write_dbuf()
    }

    /// Clears and writes a bland display buffer, turns off dimming display and oscillator.
    /// 
    /// Almost the reverse off `power_on()`.
    pub fn shutdown(&mut self) -> Result<E> {
        self.clear_dbuf();
        self.write_dbuf()?;
        self.write_dimming(Pulse::MAX)?;
        self.write_display(Display::Off)?;
        self.write_system(System::StandBy)
    }

    /// Destroys self and returns internal i2c interface.
    pub fn destroy(self) -> I2C {self.i2c}

    /// Writes unchecked slice to controller.
    /// 
    /// Use with caution! See the source code of this library of how to use it.
    pub unsafe fn write(&mut self, slice: &[u8]) -> Result<E> {
        self.i2c.write(self.address, slice)
    }

    /// Returns a copy of the address.
    pub fn address(&self) -> u8 {self.address}

    /// Sets the address.
    pub fn set_address(&mut self, address: u8) {self.address = address}
    
    /// Returns reference to internat system mode. 
    /// Might not reflect controller.
    pub fn system(&self) -> System {self.system}

    /// Writes new System mode to controller
    /// and if successful store it's new state.
    pub fn write_system(&mut self, mode: System) -> Result<E> {
        unsafe { self.write(&[SYSTEM_SETUP | mode as u8])?; }
        self.system = mode; Ok(())
    }

    /// Returns reference to internal display state. 
    /// Might not reflect controller.
    pub fn display(&self) -> Display {self.display}

    /// Writes a new display state to controller 
    /// and if successful store it's new state.
    pub fn write_display(&mut self, display: Display) -> Result<E> {
        unsafe { self.write(&[DISPLAY_SETUP | display as u8])?; }
        self.display = display; Ok(())
    }

    /// Returns reference to internal rowint state. 
    /// Might not reflect controller.
    pub fn rowint(&self) -> RowInt {self.rowint}

    /// Writes new Row/Int output to controller
    /// and if successful store it's new state.
    pub fn write_rowint(&mut self, rowint: RowInt) -> Result<E> {
        unsafe { self.write(&[ROWINT_SET | rowint as u8])?; }
        self.rowint = rowint; Ok(())
    }
    
    /// Returns dimming level. 
    /// Might not reflect controller.
    pub fn dimming(&self) -> Pulse {self.dimming}

    /// Writes a new dimming level to controller
    /// and if successful store it's new state.
    pub fn write_dimming(&mut self, pulse: Pulse) -> Result<E> {
        unsafe { self.write(&[DIMMING_SET | pulse as u8])?; }
        self.dimming = pulse; Ok(())
    }

    /// Returns display buffer as a slice.
    pub fn dbuf(&self) -> &[u8] {&self.dbuf[1..]}

    /// Returns display buffer as a mutable slice.
    pub fn dbuf_mut(&mut self) -> &mut [u8] {&mut self.dbuf[1..]}

    /// Sets the display buffer.
    ///
    /// The length of `buf` must be `SEGMENTS_SIZE`.
    ///
    /// # Panics
    ///
    /// This function will panic if the length of `buf` differs.
    pub fn set_dbuf(&mut self, buf: &[u8]) {
        self.dbuf[1..].clone_from_slice(&buf)
    }

    /// Clears Display Buffer.
    pub fn clear_dbuf(&mut self) {self.dbuf[1..].fill(0)}

    /// Writes Display Buffer to controller Display Ram.
    pub fn write_dbuf(&mut self) -> Result<E> {
        self.i2c.write(self.address, &self.dbuf)
    }

    /// Reads Display Ram from controller into a provided buffer or 
    /// internal display buffer.
    pub fn read_dram(&mut self, buf: Option<&mut [u8]>) -> Result<E> {
        self.i2c.read(self.address, buf.unwrap_or(&mut self.dbuf[1..]))
    }

    /// Writes slice to controller Display Ram 
    /// starting with from DisplayData address.
    /// 
    /// Slice should not be larger than `SEGMENTS_SIZE` as entries after will be
    /// ignored.
    pub fn write_dram(&mut self, address: &DDataAddress, slice: &[u8]) -> Result<E> {
        let mut write_buffer = [0; SEGMENTS_SIZE + 1];
        write_buffer[0] = *address as u8;
        write_buffer[1..].clone_from_slice(slice);
        unsafe { self.write(&write_buffer) }
    }
}
