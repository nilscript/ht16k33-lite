#![allow(clippy::manual_range_contains)]
#![allow(non_upper_case_globals)]
#![cfg_attr(not(feature = "std"), no_std)]

//! # ht16k33-lite
//! ht16k33 is a low level library for communicating with ht16k33 controllers.
//! https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf

use embedded_hal::blocking::i2c;
use num_derive::{FromPrimitive, ToPrimitive};
use num_traits::{FromPrimitive};

pub const SEGMENTS_SIZE: usize = 16;
pub const COMMONS_SIZE: usize  = 8;

pub trait Command { const COMMAND_MASK: isize; }

/// System Setup Register.
#[derive(Copy, Clone, Debug, Hash, FromPrimitive, ToPrimitive)]
pub enum SystemSetupRegister {
    /// Oscillator off
    StandBy = Self::COMMAND_MASK | 0,
    /// Oscillator on
    Normal  = Self::COMMAND_MASK | 1,
}

impl Command for SystemSetupRegister {
    /// Write only
    const COMMAND_MASK: isize = 0b0010_0000; 
}

/// Alias for System Setup Register.
pub type System = SystemSetupRegister;

/// ROW/INT Setup Register.
#[derive(Copy, Clone, Debug, Hash, FromPrimitive, ToPrimitive)]
pub enum RowIntSetupRegister {
    /// Row driver output
    Row     = Self::COMMAND_MASK | 0b0000,
    /// Int output, active low
    IntLow  = Self::COMMAND_MASK | 0b0001,
    /// Int output, active high
    IntHigh = Self::COMMAND_MASK | 0b0011,
}

impl Command for RowIntSetupRegister {
    /// Write only
    const COMMAND_MASK: isize = 0b1010_0000;
}

/// Alias for ROW/INT Setup Register.
pub type RowInt = RowIntSetupRegister;

/// Display Setup Register.
#[derive(Copy, Clone, Debug, Hash, FromPrimitive, ToPrimitive)]
pub enum DisplaySetupRegister {
    /// Display off
    Off     = Self::COMMAND_MASK | 0b0000,
    /// Display on
    On      = Self::COMMAND_MASK | 0b0001,
    /// Display on + Blinking frequency = 2hz
    TwoHz   = Self::COMMAND_MASK | 0b0011,
    /// Display on + Blinking frequency = 1hz
    OneHz   = Self::COMMAND_MASK | 0b0101,
    /// Display on + Blinking frequency = 0.5hz
    HalfHz  = Self::COMMAND_MASK | 0b0111,
}

impl Command for DisplaySetupRegister {
    /// Write only
    const COMMAND_MASK: isize = 0b1000_0000; 
}

/// Alias for Display Setup Register.
pub type Display = DisplaySetupRegister;

/// Digital Dimming Data Input Pulse Width Duties.
#[derive(Copy, Clone, Debug, Hash, FromPrimitive, ToPrimitive)]
pub enum DigitalDimmingDataInput { 
    Duty1_16 =  Self::COMMAND_MASK | 0b0000,
    Duty2_16 =  Self::COMMAND_MASK | 0b0001,
    Duty3_16 =  Self::COMMAND_MASK | 0b0010,
    Duty4_16 =  Self::COMMAND_MASK | 0b0011,
    Duty5_16 =  Self::COMMAND_MASK | 0b0100,
    Duty6_16 =  Self::COMMAND_MASK | 0b0101,
    Duty7_16 =  Self::COMMAND_MASK | 0b0110,
    Duty8_16 =  Self::COMMAND_MASK | 0b0111,
    Duty9_16 =  Self::COMMAND_MASK | 0b1000,
    Duty10_16 = Self::COMMAND_MASK | 0b1001,
    Duty11_16 = Self::COMMAND_MASK | 0b1010,
    Duty12_16 = Self::COMMAND_MASK | 0b1011,
    Duty13_16 = Self::COMMAND_MASK | 0b1100,
    Duty14_16 = Self::COMMAND_MASK | 0b1101,
    Duty15_16 = Self::COMMAND_MASK | 0b1110,
    Duty16_16 = Self::COMMAND_MASK | 0b1111,
}

impl Command for DigitalDimmingDataInput {
    /// Write only
    const COMMAND_MASK: isize = 0b1110_0000;
}

/// Alias for Digital Dimming Data Input.
pub type Dimming = DigitalDimmingDataInput;


/// Display Data Address Pointer.
#[derive(Copy, Clone, Debug, Hash, FromPrimitive, ToPrimitive)]
pub enum DisplayDataAddressPointer {
    Addr0 =  Self::COMMAND_MASK | 0b0000,
    Addr1 =  Self::COMMAND_MASK | 0b0001,
    Addr2 =  Self::COMMAND_MASK | 0b0010,
    Addr3 =  Self::COMMAND_MASK | 0b0011,
    Addr4 =  Self::COMMAND_MASK | 0b0100,
    Addr5 =  Self::COMMAND_MASK | 0b0101,
    Addr6 =  Self::COMMAND_MASK | 0b0110,
    Addr7 =  Self::COMMAND_MASK | 0b0111,
    Addr8 =  Self::COMMAND_MASK | 0b1000,
    Addr9 =  Self::COMMAND_MASK | 0b1001,
    Addr10 = Self::COMMAND_MASK | 0b1010,
    Addr11 = Self::COMMAND_MASK | 0b1011,
    Addr12 = Self::COMMAND_MASK | 0b1100,
    Addr13 = Self::COMMAND_MASK | 0b1101,
    Addr14 = Self::COMMAND_MASK | 0b1110,
    Addr15 = Self::COMMAND_MASK | 0b1111,
}

impl Command for DisplayDataAddressPointer {
    /// R/W;
    const COMMAND_MASK: isize = 0b0000_0000;
}

/// Alias for Display Data Address Pointer
pub type DDAP = DisplayDataAddressPointer;

/// Key Data Address Pointers.
#[derive(Copy, Clone, Debug, Hash, FromPrimitive, ToPrimitive)]
pub enum KeyDataAddressPointer {
    Addr0 = Self::COMMAND_MASK | 0b000,
    Addr1 = Self::COMMAND_MASK | 0b001,
    Addr2 = Self::COMMAND_MASK | 0b010,
    Addr3 = Self::COMMAND_MASK | 0b011,
    Addr4 = Self::COMMAND_MASK | 0b100,
    Addr5 = Self::COMMAND_MASK | 0b101,
    Addr6 = Self::COMMAND_MASK | 0b110,
    Addr7 = Self::COMMAND_MASK | 0b111,
}

impl Command for KeyDataAddressPointer {
    /// Read only
    const COMMAND_MASK: isize = 0b0100_0000;
}

/// Alias for Key Data Address Pointer.
pub type KDAP = KeyDataAddressPointer;

/// Int Flag Address Pointer.
/// 
/// Not yet implemented in `struct HT16K33` as I don't understand how it's used 
/// or for what.
pub const IntFlagAddressPointer: isize = 0b0110_0000;

/// Alias for Int Flag Address Pointer.
pub const IFAP: isize = IntFlagAddressPointer;

/// Test Mode.
/// 
/// Not yet implemented in `struct HT16K33` as I don't know how it's used.
pub const TestModeHoltekOnly: isize = 0b1101_1001;

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
    i2c_address:  u8,
    dbuf:     [u8; SEGMENTS_SIZE + 1],
    system:   SystemSetupRegister,
    display:  DisplaySetupRegister,
    rowint:   RowIntSetupRegister,
    dimming:  DigitalDimmingDataInput,
}

impl<I2C, E> HT16K33<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E>
{
    /// Creates a new instance.
    /// Should be followed by power_on().
    pub fn new(i2c: I2C, i2c_address: u8) -> Self {
        let mut dbuf = [0; SEGMENTS_SIZE + 1];
        dbuf[0] = DisplayDataAddressPointer::Addr0 as u8;

        HT16K33 {
            i2c,
            i2c_address,
            dbuf,
            system:   SystemSetupRegister::StandBy,
            display:  DisplaySetupRegister::Off,
            rowint:   RowIntSetupRegister::Row,
            dimming:  DigitalDimmingDataInput::Duty16_16,
        }
    }

    /// Turns system to normal mode, set's Row/Int to Row, turns on display, 
    /// turns dimming to max, and writes what is in display buffer.
    pub fn power_on(&mut self) -> Result<E> {
        self.write_system(SystemSetupRegister::Normal)?;
        self.write_rowint(RowIntSetupRegister::Row)?;
        self.write_display(DisplaySetupRegister::On)?;
        self.write_dimming(DigitalDimmingDataInput::Duty16_16)?;
        self.clear_dbuf();
        self.write_dbuf()
    }

    /// Clears and writes a bland display buffer, turns off dimming display and 
    /// oscillator.
    /// 
    /// Almost the reverse off `power_on()`.
    pub fn shutdown(&mut self) -> Result<E> {
        self.clear_dbuf();
        self.write_dbuf()?;
        self.write_dimming(DigitalDimmingDataInput::Duty16_16)?;
        self.write_display(DisplaySetupRegister::Off)?;
        self.write_system(SystemSetupRegister::StandBy)
    }

    /// Destroys self and returns internal i2c interface.
    pub fn i2c(self) -> I2C {self.i2c}

    /// Writes unchecked slice to controller.
    /// 
    /// Use with caution! See the source code of this library of how to use it.
    pub unsafe fn write(&mut self, slice: &[u8]) -> Result<E> {
        self.i2c.write(self.i2c_address, slice)
    }

    /// Returns a copy of the i2c address.
    pub fn i2c_address(&self) -> u8 {self.i2c_address}

    /// Sets the i2c address.
    pub fn set_i2c_address(&mut self, i2c_address: u8) {
        self.i2c_address = i2c_address
    }
    
    /// Returns reference to internat system mode. 
    /// Might not reflect controller.
    pub fn system(&self) -> SystemSetupRegister {self.system}

    /// Writes new System mode to controller
    /// and if successful store it's new state.
    pub fn write_system(&mut self, mode: SystemSetupRegister) -> Result<E> {
        unsafe { self.write(&[mode as u8])?; }
        self.system = mode; Ok(())
    }

    /// Returns reference to internal display state. 
    /// Might not reflect controller.
    pub fn display(&self) -> DisplaySetupRegister {self.display}

    /// Writes a new display state to controller 
    /// and if successful store it's new state.
    pub fn write_display(&mut self, dsr: DisplaySetupRegister) -> Result<E> {
        unsafe { self.write(&[dsr as u8])?; }
        self.display = dsr; Ok(())
    }

    /// Returns reference to internal rowint state. 
    /// Might not reflect controller.
    pub fn rowint(&self) -> RowInt {self.rowint}

    /// Writes new Row/Int output to controller
    /// and if successful store it's new state.
    pub fn write_rowint(&mut self, rowint: RowIntSetupRegister) -> Result<E> {
        unsafe { self.write(&[rowint as u8])?; }
        self.rowint = rowint; Ok(())
    }
    
    /// Returns dimming level. 
    /// Might not reflect controller.
    pub fn dimming(&self) -> DigitalDimmingDataInput {self.dimming}

    /// Writes a new dimming level to controller
    /// and if successful store it's new state.
    pub fn write_dimming(&mut self, dim: DigitalDimmingDataInput) -> Result<E> {
        unsafe { self.write(&[dim as u8])?; }
        self.dimming = dim; Ok(())
    }

    /// Returns display data address pointer.
    pub fn display_data_address_pointer(&self) -> DisplayDataAddressPointer {
        DisplayDataAddressPointer::from_u8(self.dbuf[0])
            .expect("Internal Display Buffer has been corrupted.")
    }

    /// Sets the display data address pointer.
    pub fn set_display_data_address_pointer(
        &mut self, 
        ddap: DisplayDataAddressPointer
    ) {self.dbuf[0] = ddap as u8}

    /// Returns display buffer as a slice.
    pub fn dbuf(&self) -> &[u8] {&self.dbuf[1..]}

    /// Returns display buffer as a mutable slice.
    pub fn dbuf_mut(&mut self) -> &mut [u8] {&mut self.dbuf[1..]}

    /// Sets the display buffer.
    pub fn set_dbuf(&mut self, array: &[u8; SEGMENTS_SIZE]) {
        self.dbuf[1..].clone_from_slice(array)
    }

    /// Clears Display Buffer.
    pub fn clear_dbuf(&mut self) {self.dbuf[1..].fill(0)}

    /// Writes Display Buffer to controller Display Ram.
    pub fn write_dbuf(&mut self) -> Result<E> {
        self.i2c.write(self.i2c_address, &self.dbuf)
    }

    /// Reads Display Ram from controller into a provided buffer or into the
    /// internal display buffer.
    pub fn read_dram(&mut self, buf: Option<&mut [u8]>) -> Result<E> {
        self.i2c.read(self.i2c_address, buf.unwrap_or(&mut self.dbuf[1..]))
    }


    /// Writes slice to controller Display Ram starting from the 
    /// Display Data Address Pointer. Slice can be up to `SEGMENTS_SIZE` long.
    /// No state in `self` is mutated except for `self.i2c` (driver).
    /// 
    /// # Panic
    /// 
    /// Panics if slice is longer than `SEGMENTS_SIZE`.
    pub fn write_dram(&mut self, ddap: DisplayDataAddressPointer, slice: &[u8]) 
    -> Result<E> {
        let mut buf = [0; SEGMENTS_SIZE + 1];
        buf[0] = ddap as u8;
        buf[1..=slice.len()].clone_from_slice(slice);
        self.i2c.write(self.i2c_address, &buf[..=slice.len()])
    }
}
