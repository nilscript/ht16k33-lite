#![allow(clippy::manual_range_contains)]
#![cfg_attr(not(feature = "std"), no_std)]
#![feature(step_trait)]

//! 

use embedded_hal::blocking::i2c::{Read, Write};

pub const SEGMENTS_SIZE: usize  = 16;
pub const COMMONS_SIZE: usize   = 8;

/// Contains all commands for each subsystem.
pub mod command {
    pub const DISPLAY_DATA_ADDRESS_POINTER: u8  = 0b0000_0000; // R/W
    pub const SYSTEM_SETUP: u8                  = 0b0010_0000; // Write only
    pub const KEY_DATA_ADDRESS_POINTER: u8      = 0b0100_0000; // Read only
    pub const INT_FLAG_ADDRESS_POINTER: u8      = 0b0110_0000; // Read only
    pub const DISPLAY_SETUP: u8                 = 0b1000_0000; // Write only
    pub const ROWINT_SET: u8                    = 0b1010_0000; // Write only
    pub const DIMMING_SET: u8                   = 0b1110_0000; // Write only
    pub const TEST_MODE_HOLTEK_ONLY: u8         = 0b1101_1001; // Write only
}

/// Contains all legal states for each subsystem.
pub mod state {
    /// System Setup Register States.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum System {
        /// Oscillator off
        StandBy = 0,
        /// Oscillator on
        Normal  = 1,
    }

    /// ROW/INT Set Register State.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum RowInt {
        /// Row driver output
        Row     = 0b0000_0000,
        /// Int output, active low
        IntLow  = 0b0000_0001,
        /// Int ooutput, active high
        IntHigh = 0b0000_0011,
    }

    /// Display Setup Register State.
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

    /// Digital Dimming Data Input Pulse Width Duty.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum Pulse { 
        /// 1/16 Duty
        Duty1,
        /// 2/16 Duty
        Duty2,
        /// 3/16 Duty
        Duty3,
        /// 4/16 Duty
        Duty4,
        /// 5/16 Duty
        Duty5,
        /// 6/16 Duty
        Duty6,
        /// 7/16 Duty
        Duty7,
        /// 8/16 Duty
        Duty8,
        /// 9/16 Duty
        Duty9,
        /// 10/16 Duty
        Duty10,
        /// 11/16 Duty
        Duty11,
        /// 12/16 Duty
        Duty12,
        /// 13/16 Duty
        Duty13,
        /// 14/16 Duty
        Duty14,
        /// 15/16 Duty
        Duty15,
        /// 16/16 Duty
        Duty16, 
    }
}

pub mod address {
    /// Display Data Address Pointer.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum DisplayDataAddress {
        Pointer0,
        Pointer1,
        Pointer2,
        Pointer3,
        Pointer4,
        Pointer5,
        Pointer6,
        Pointer7,
        Pointer8,
        Pointer9,
        Pointer10,
        Pointer11,
        Pointer12,
        Pointer13,
        Pointer14,
        Pointer15,
    }

    /// Key Data Address Pointer.
    #[derive(Copy, Clone, Debug, Hash)]
    pub enum KeyDataAddress {  
        Pointer0,
        Pointer1,
        Pointer2,
        Pointer3,
        Pointer4,
        Pointer5,
    }
}

use command::*;
use state::*;
type Result<Error> = core::result::Result<(), Error>;

/// Driver for the HT16K33 RAM Mapping 16*8 LED controller Driver with keyscan.
/// 
/// Has an internal state which it tries to reflect
/// onto the connected controller.
/// 
/// Internal buffer is exposed and should be read and edited that way.
/// Write buffer with 
#[derive(Copy, Clone, Debug, Hash)]
pub struct HT16K33<I2C> {
    i2c:     I2C,
    address: u8,

    pub dbuf: [u8; SEGMENTS_SIZE],

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
        HT16K33 {
            address,
            i2c,

            dbuf: [0; SEGMENTS_SIZE],

            system:   System::StandBy,
            display:  Display::Off,
            rowint:   RowInt::Row,
            dimming:  Pulse::Duty16,
        }
    }

    /// Turns system to normal mode, set's Row/Int to Row, turns on display, 
    /// turns dimming to max, and writes a blank dbuffer.
    pub fn power_on(&mut self) -> Result<E> {
        self.write_system(System::Normal)?;
        self.write_rowint(RowInt::Row)?;
        self.write_display(Display::On)?;
        self.write_dimming(Pulse::Duty16)?;
        self.write_dbuf()
    }

    /// Writes blank buffer, turns off dimming display and oscillator.
    /// 
    /// Almost the reverse off `power_on()`.
    pub fn shutdown(&mut self) -> Result<E> {
        self.write_dbuf()?;
        self.write_dimming(Pulse::Duty16)?;
        self.write_display(Display::Off)?;
        self.write_system(System::StandBy)
    }

    /// Destroys self and returns internal i2c interface.
    pub fn destroy(self) -> I2C {self.i2c}

    /// Writes unchecked slice to controller.
    /// 
    /// Use with caution! See the sourcecode of this library of how to use it.
    pub unsafe fn write(&mut self, slice: &[u8]) -> Result<E> {
        self.i2c.write(self.address, slice)
    }
    
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

    /// Clears Display Buffer.
    pub fn clear_dbuf(&mut self) {
        self.dbuf = [0; SEGMENTS_SIZE];
    }

    /// Writes Display Buffer to controller Display Ram.
    pub fn write_dbuf(&mut self) -> Result<E> {
        let mut write_buffer = [0; SEGMENTS_SIZE + 1];
        write_buffer[1..].clone_from_slice(&self.dbuf);
        unsafe { self.write(&write_buffer) }
    }

    /// Reads Display Ram from controller into buffer.
    pub fn read_dram(&mut self, buf: &mut [u8]) -> Result<E> {
        self.i2c.read(self.address, buf)
    }

    /// Writes slice to controller Display Ram 
    /// starting with from DisplayData address.
    /// 
    /// Slice should not be larger than `SEGMENTS_SIZE` as entries after will be
    /// ignored.
    pub fn write_dram(&mut self, address: &address::DisplayDataAddress, slice: &[u8]) -> Result<E> {
        let mut write_buffer = [0; SEGMENTS_SIZE + 1];
        write_buffer[0] = *address as u8;
        write_buffer[1..].clone_from_slice(slice);
        unsafe { self.write(&write_buffer) }
    }
}
