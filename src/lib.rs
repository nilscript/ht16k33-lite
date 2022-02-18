//! # ht16k33-lite
//! 
//! A driver for [Holtek HT16K33 RAM Mapping 16*8 LED Controller Driver with keyscan](https://www.holtek.com/productdetail/-/vg/HT16K33) 
//! which aims to be a simple implementation and to only implement the bare 
//! minimum to cover all basic use cases.
//! 
//! Using this library should be done after reading the documentation for the
//! ht16k33 Controller as this library.

pub mod prelude {
    pub use super::DisplayDataAddressPointer as DDAP;
    pub use super::SystemSetup as SSetup;
    pub use super::KeyDataAddressPointer as KDAP;
    pub use super::DisplaySetup as DSetup;
    pub use super::RowIntSet as RIS;
    pub use super::DimmingSet as DIS;
    pub use super::HT16K33;
}

use embedded_hal::blocking::i2c::{Read, Write};

pub const SEGMENTS_SIZE: usize  = 16;
pub const COMMONS_SIZE: usize   = 8;


pub const DISPLAY_DATA_ADDRESS_POINTER: u8 = 0b0000_0000;

/// Read / Write
#[derive(Copy, Clone, Debug, Hash)]
pub enum DisplayDataAddressPointer {
    P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15
}

/// Command address for System Setup
pub const SYSTEM_SETUP: u8 = 0b0010_0000;

/// Command for System Setup
/// 
/// Option: Write only
#[derive(Copy, Clone, Debug, Hash)]
pub enum SystemSetup {
    /// Oscillator off
    StandBy = 0,
    /// Oscillator on
    Normal  = 1,
}

/// Command address for Key Data Address Pointer
pub const KEY_DATA_ADDRESS_POINTER: u8 = 0b0100_0000;

/// Command for Key Data Address Pointer
/// 
/// Option: Read only
#[derive(Copy, Clone, Debug, Hash)]
pub enum KeyDataAddressPointer {  
    P0, P1, P2, P3, P4, P5
}

/// Command for Int Flag Address Pointer
/// 
/// Option: Read only
pub const INT_FLAG_ADDRESS_POINTER: u8 = 0b0110_0000;

/// Command address for Display Setup
pub const DISPLAY_SETUP: u8 = 0b1000_0000;

/// Command 
/// 
/// Option: Write only
#[derive(Copy, Clone, Debug, Hash)]
pub enum DisplaySetup {
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

/// Write only
pub const ROWINT_SET: u8 = 0b1010_0000;

#[derive(Copy, Clone, Debug, Hash)]
pub enum RowIntSet {
    /// Row driver output
    Row     = 0b0000_0000,
    /// Int output, active low
    IntLow  = 0b0000_0001,
    /// Int output, active high
    IntHigh = 0b0000_0011,
}

/// Write only
pub const DIMMING_SET: u8 = 0b1110_0000;

/// Digital Dimming Data Input Pulse Width Duty.
#[derive(Copy, Clone, Debug, Hash)]
pub enum DimmingSet { 
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

/// Write only
pub const TEST_MODE_HOLTEK_ONLY: u8 = 0b1101_1001;

pub type Result<Error> = core::result::Result<(), Error>;

/// Driver for the HT16K33 RAM Mapping 16*8 LED controller Driver with keyscan.
/// 
/// Has an internal state which it tries to reflect
/// onto the connected controller.
/// 
/// Internal buffer is exposed and should be read and edited that way.
/// Write buffer with 
#[derive(Copy, Clone, Debug, Hash)]
pub struct HT16K33<I2C> {
    i2c:        I2C,
    addr:       u8,

    pub dbuf:   [u8; SEGMENTS_SIZE],

    sys:        SystemSetup,
    dpy:        DisplaySetup,
    rowint:     RowIntSet,
    dim:        DimmingSet,
}

impl<I2C, E> HT16K33<I2C>
where
    I2C: Read<Error = E> + Write<Error = E>
{
    /// Creates a new instance.
    pub fn new(i2c: I2C, addr: u8) -> Self {
        HT16K33 {
            addr,
            i2c,

            dbuf:   [0; SEGMENTS_SIZE],

            sys:    SystemSetup::StandBy,
            dpy:    DisplaySetup::Off,
            rowint: RowIntSet::Row,
            dim:    DimmingSet::Duty16,
        }
    }

    /// Turns system to normal mode, set's Row/Int to Row, turns on display, 
    /// turns dimming to max, and writes a blank dbuf.
    pub fn power_on(&mut self) -> Result<E> {
        self.write_system_setup(SystemSetup::Normal)?;
        self.write_row_int_set(RowIntSet::Row)?;
        self.write_display_setup(DisplaySetup::On)?;
        self.write_dimming_set(DimmingSet::Duty16)?;
        self.clear_dbuf();
        self.write_dbuf()
    }

    /// Writes blank buffer, turns off display and oscillator.
    pub fn shutdown(&mut self) -> Result<E> {
        self.clear_dbuf();
        self.write_dbuf()?;
        self.write_display_setup(DisplaySetup::Off)?;
        self.write_system_setup(SystemSetup::StandBy)
    }

    /// Destroys self and returns internal i2c interface.
    pub fn destroy(self) -> I2C {self.i2c}

    /// Writes unchecked slice to controller.
    /// 
    /// Use with caution! See the source code of this library for usage examples.
    pub unsafe fn write_raw(&mut self, slice: &[u8]) -> Result<E> {
        self.i2c.write(self.addr, slice)
    }
    
    /// Returns reference to internat system mode. 
    pub fn read_system_setup(&self) -> SystemSetup {self.sys}

    /// Writes new System mode to controller
    /// and if successful store it's new state.
    pub fn write_system_setup(&mut self, sys: SystemSetup) -> Result<E> {
        unsafe { self.write_raw(&[SYSTEM_SETUP | sys as u8])?; }
        self.sys = sys; Ok(())
    }

    /// Returns reference to internal display state. 
    pub fn read_display_setup(&self) -> DisplaySetup {self.dpy}

    /// Writes a new display state to controller 
    /// and if successful store it's new state.
    pub fn write_display_setup(&mut self, dpy: DisplaySetup) -> Result<E> {
        unsafe { self.write_raw(&[DISPLAY_SETUP | dpy as u8])?; }
        self.dpy = dpy; Ok(())
    }

    /// Returns reference to internal rowint state. 
    pub fn read_row_int_set(&self) -> RowIntSet {self.rowint}

    /// Writes new Row/Int output to controller
    /// and if successful store it's new state.
    pub fn write_row_int_set(&mut self, rowint: RowIntSet) -> Result<E> {
        unsafe { self.write_raw(&[ROWINT_SET | rowint as u8])?; }
        self.rowint = rowint; Ok(())
    }
    
    /// Returns dimming level. 
    pub fn read_dimming_set(&self) -> DimmingSet {self.dim}

    /// Writes a new dimming level to controller
    /// and if successful store it's new state.
    pub fn write_dimming_set(&mut self, dim: DimmingSet) -> Result<E> {
        unsafe { self.write_raw(&[DIMMING_SET | dim as u8])?; }
        self.dim = dim; Ok(())
    }

    /// Clears Display Buffer.
    pub fn clear_dbuf(&mut self) {
        self.dbuf = [0; SEGMENTS_SIZE];
    }

    /// Writes Display Buffer to controller Display Ram.
    pub fn write_dbuf(&mut self) -> Result<E> {
        let mut write_buffer = [0; SEGMENTS_SIZE + 1];
        write_buffer[1..].clone_from_slice(&self.dbuf);
        unsafe { self.write_raw(&write_buffer) }
    }

    /// Reads Display Ram from controller into buffer.
    /// 
    pub fn read_dram(&mut self, buf: &mut [u8]) -> Result<E> {
        self.i2c.read(self.addr, buf)
    }

    /// Writes slice to controller Display Ram 
    /// starting with from DisplayData address.
    /// 
    /// This will update only addresses including and after `addr` so less
    /// the whole ram need not to be updated on a write
    /// 
    /// Slice should not be larger than `SEGMENTS_SIZE - addr` as entries after will be
    /// ignored.
    pub fn write_dram(&mut self, addr: &DisplayDataAddressPointer, slice: &[u8]) -> Result<E> {
        let mut write_buffer = [0; SEGMENTS_SIZE + 1];
        write_buffer[0] = *addr as u8;
        write_buffer[1..].clone_from_slice(slice);
        unsafe { self.write_raw(&write_buffer) }
    }
}
