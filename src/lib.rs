#![allow(clippy::manual_range_contains)]
#![allow(non_upper_case_globals)]
#![cfg_attr(not(feature = "std"), no_std)]

//! ht16k33-lite is a low level library for communicating with ht16k33
//! controllers.
//!
//! <https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf>

pub use embedded_hal::blocking::i2c::{self, Read, Write};
pub use enum_index::IndexEnum;
use enum_index_derive::{EnumIndex, IndexEnum};

pub const SEGMENTS_SIZE: usize = 16;
pub const COMMONS_SIZE: usize  = 8;

/// Trait for holding a command mask.
///
/// See <https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf#HT16K33v110_110516.indd%3A.134493>
/// for more information.
pub trait Command { const COMMAND_MASK: u8; }

/// [System Setup Register](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf#HT16K33v110_110516.indd%3A.134352)
#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
#[repr(u8)]
pub enum SystemSetupRegister {
    /// Oscillator off
    StandBy = Self::COMMAND_MASK | 0,
    /// Oscillator on
    Normal  = Self::COMMAND_MASK | 1,
}

impl Command for SystemSetupRegister {
    /// Write only
    const COMMAND_MASK: u8 = 0b0010_0000; 
}

/// Alias for System Setup Register.
pub type System = SystemSetupRegister;

/// [ROW/INT Set Register](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf#HT16K33v110_110516.indd%3A.134356)
#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
#[repr(u8)]
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
    const COMMAND_MASK: u8 = 0b1010_0000;
}

/// Alias for ROW/INT Setup Register.
pub type RowInt = RowIntSetupRegister;

/// [Display Setup Register](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf#HT16K33v110_110516.indd%3A.134360)
#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
#[repr(u8)]
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
    const COMMAND_MASK: u8 = 0b1000_0000; 
}

/// Alias for Display Setup Register.
pub type Display = DisplaySetupRegister;

/// [Digital Dimming Data Input](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf#HT16K33v110_110516.indd%3A.134397)
#[derive(Copy, Clone, Eq, PartialEq, EnumIndex, IndexEnum, Hash, PartialOrd, Ord, Debug)]
#[repr(u8)]
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
    const COMMAND_MASK: u8 = 0b1110_0000;
}

/// Alias for Digital Dimming Data Input.
pub type Dimming = DigitalDimmingDataInput;

/// [Display Data Address Pointer](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf#HT16K33v110_110516.indd%3A.134369)
#[derive(Copy, Clone, Eq, PartialEq, EnumIndex, IndexEnum, Hash, PartialOrd, Ord, Debug)]
#[repr(u8)]
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
    const COMMAND_MASK: u8 = 0b0000_0000;
}

/// Alias for Display Data Address Pointer
pub type DDAP = DisplayDataAddressPointer;

/// [Key Data Address Pointers](https://cdn-shop.adafruit.com/datasheets/ht16K33v110.pdf#HT16K33v110_110516.indd%3A.134373)
#[derive(Copy, Clone, Eq, PartialEq, EnumIndex, IndexEnum, Hash, PartialOrd, Ord, Debug)]
#[repr(u8)]
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
    const COMMAND_MASK: u8 = 0b0100_0000;
}

/// Alias for Key Data Address Pointer.
pub type KDAP = KeyDataAddressPointer;

/// Int Flag Address Pointer.
/// 
/// No implementation is planned. PRs welcomed.
pub const INT_FLAG_ADDRESS_POINTER: u8 = 0b0110_0000;

/// Alias for Int Flag Address Pointer.
pub const IFAP: u8 = INT_FLAG_ADDRESS_POINTER;

/// Test Mode.
/// 
/// No implementation is planned. PRs welcomed.
pub const TEST_MODE_HOLTEK_ONLY: u8 = 0b1101_1001;

pub type Result<Error> = core::result::Result<(), Error>;

/// Driver for the HT16K33 RAM Mapping 16*8 LED controller Driver with key scan.
/// 
/// Has an internal state which it tries to reflect
/// onto the connected controller.
#[derive(Copy, Clone, Debug, Hash)]
pub struct HT16K33<I2C> {
    i2c:            I2C,
    i2c_address:    u8,
    // Contains both the `DisplayDataAddressPointer` as index 0 and the
    // display buffer on the remaining 16 indexes.
    dbuf:           [u8; SEGMENTS_SIZE + 1],
    system:         SystemSetupRegister,
    display:        DisplaySetupRegister,
    rowint:         RowIntSetupRegister,
    dimming:        DigitalDimmingDataInput,
}

impl<I2C> HT16K33<I2C> {
    /// Creates a new instance of HT16K33 driver.
    /// Nothing is known of the state of the controller so it is best to reset
    /// it with `.power_on()` before use.
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
}

pub trait HT16K33Trait<I2C, E>: Read + Write {
    /// Destroys self and returns internal i2c interface.
    fn i2c(self) -> I2C;

    /// Returns a copy of the i2c address.
    fn i2c_address(&self) -> u8;

    /// Sets the i2c address.
    fn set_i2c_address(&mut self, i2c_address: u8);
    
    /// Returns internal system setup register state. 
    /// Might not reflect the state of the controller.
    fn system(&self) -> SystemSetupRegister;

    /// Writes system setup register state to controller
    /// and if successful store it's new state internally.
    fn write_system(&mut self, system: SystemSetupRegister) -> Result<E>;

    /// Returns internal display setup register state. 
    /// Might not reflect the state of the controller.
    fn display(&self) -> DisplaySetupRegister;

    /// Writes display setup register state to controller 
    /// and if successful store it's new state internally.
    fn write_display(&mut self, dsr: DisplaySetupRegister) -> Result<E>;

    /// Returns internal ROW/INT setup register state. 
    /// Might not reflect the state of the controller.
    fn rowint(&self) -> RowIntSetupRegister;

    /// Writes ROW/INT setup register state to controller
    /// and if successful store it's new state internally.
    fn write_rowint(&mut self, rowint: RowIntSetupRegister) -> Result<E>;

    /// Returns internal digital dimming data input state. 
    /// Might not reflect the state of the controller.
    fn dimming(&self) -> DigitalDimmingDataInput;

    /// Writes digital dimming data input state to controller
    /// and if successful store it's new state internally.
    fn write_dimming(&mut self, dim: DigitalDimmingDataInput) -> Result<E>;

    /// Returns the internal display data address pointer state.
    /// Might not reflect the state of the controller.
    fn display_data_address_pointer(&self) -> DisplayDataAddressPointer;

    /// Sets the display data address pointer internally.
    fn set_display_data_address_pointer(
        &mut self,
        ddap: DisplayDataAddressPointer
    );

    /// Returns display buffer as a slice.
    fn dbuf(&self) -> &[u8];

    /// Returns display buffer as a mutable slice.
    fn dbuf_mut(&mut self) -> &mut [u8];

    /// Sets display buffer from a sized array.
    fn set_dbuf(&mut self, src: &[u8; SEGMENTS_SIZE]);

    /// Clears display buffer by setting all values to zero.
    fn clear_dbuf(&mut self);

    /// Writes display buffer to controller display ram.
    fn write_dbuf(&mut self) -> Result<E>;

    /// Reads display ram from controller into a provided buffer.
    fn read_dram(&mut self, buf: &mut [u8]) -> Result<E>;

    /// Reads display ram from controller into the internal display buffer.
    fn read_dram_to_dbuf(&mut self) -> Result<E>;

    /// Writes slice to controller display ram starting from the 
    /// display data address pointer. Slice can be up to `SEGMENTS_SIZE` long.
    /// No state in `self` is mutated except for `self.i2c` (internal driver).
    /// 
    /// # Panic
    /// 
    /// Panics if slice is longer than `SEGMENTS_SIZE`.
    fn write_dram(&mut self, ddap: DisplayDataAddressPointer, slice: &[u8]) 
    -> Result<E>;

    /// Clears display buffer and writes to controller display ram.
    fn clear_dram(&mut self) -> Result<E> {
        self.clear_dbuf();
        self.write_dbuf()
    }

    /// Writes `SystemSetupRegister::Normal`, 
    /// `RowIntSetupRegister::Row`, 
    /// `DisplaySetupRegister::On` and 
    /// `DigitalDimmingDataInput::Duty16_16` to controller.
    /// Then clears and writes display buffer.
    fn power_on(&mut self) -> Result<E> {
        self.write_system(SystemSetupRegister::Normal)?;
        self.write_rowint(RowIntSetupRegister::Row)?;
        self.write_display(DisplaySetupRegister::On)?;
        self.write_dimming(DigitalDimmingDataInput::Duty16_16)?;
        self.clear_dram()
    }

    /// Clears and writes a display buffer to controller,
    /// writes `DigitalDimmingDataInput::Duty16_16`,
    /// `DisplaySetupRegister::Off` and
    /// `SystemSetupRegister::StandBy` to controller.
    fn shutdown(&mut self) -> Result<E> {
        self.clear_dram()?;
        self.write_dimming(DigitalDimmingDataInput::Duty16_16)?;
        self.write_display(DisplaySetupRegister::Off)?;
        self.write_system(SystemSetupRegister::StandBy)
    }
}

impl<I2C, E> Read for HT16K33<I2C>
where
    I2C: Read<Error = E>
{
    type Error = E;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<Self::Error> {
        self.i2c.read(address, buffer)
    }
}

impl<I2C, E> Write for HT16K33<I2C>
where
    I2C: Write<Error = E>
{
    type Error = E;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<Self::Error> {
        self.i2c.write(address, bytes)
    }
}

impl<I2C, E> HT16K33Trait<I2C, E> for HT16K33<I2C>
where
    I2C: Read<Error = E> + Write<Error = E>
{
    fn i2c(self) -> I2C {self.i2c}

    fn i2c_address(&self) -> u8 {self.i2c_address}

    fn set_i2c_address(&mut self, i2c_address: u8) {
        self.i2c_address = i2c_address
    }
    
    fn system(&self) -> System {self.system}

    fn write_system(&mut self, sys: System) -> Result<E> {
        self.write(self.i2c_address, &[sys as u8])?;
        self.system = sys; Ok(())
    }

    fn display(&self) -> Display {self.display}

    fn write_display(&mut self, dpy: Display) -> Result<E> {
        self.write(self.i2c_address, &[dpy as u8])?;
        self.display = dpy; Ok(())
    }

    fn rowint(&self) -> RowInt {self.rowint}

    fn write_rowint(&mut self, rowint: RowInt) -> Result<E> {
        self.write(self.i2c_address, &[rowint as u8])?;
        self.rowint = rowint; Ok(())
    }
    
    fn dimming(&self) -> Dimming {self.dimming}

    fn write_dimming(&mut self, dim: Dimming) -> Result<E> {
        self.write(self.i2c_address, &[dim as u8])?;
        self.dimming = dim; Ok(())
    }

    fn display_data_address_pointer(&self) -> DDAP {
        DDAP::index_enum(self.dbuf[0] as usize)
            .expect("Internal Display Buffer has been corrupted.")
    }

    fn set_display_data_address_pointer(&mut self, ddap: DDAP) {
        self.dbuf[0] = ddap as u8
    }

    fn dbuf(&self) -> &[u8] {&self.dbuf[1..]}

    fn dbuf_mut(&mut self) -> &mut [u8] {&mut self.dbuf[1..]}

    fn set_dbuf(&mut self, src: &[u8; SEGMENTS_SIZE]) {
        self.dbuf[1..].clone_from_slice(src)
    }

    fn clear_dbuf(&mut self) {self.dbuf[1..].fill(0)}

    fn write_dbuf(&mut self) -> Result<E> {
        self.i2c.write(self.i2c_address, &self.dbuf)
    }

    fn read_dram(&mut self, buf: &mut [u8]) -> Result<E> {
        self.i2c.read(self.i2c_address, buf)
    }

    fn read_dram_to_dbuf(&mut self) -> Result<E> {
        self.i2c.read(self.i2c_address, &mut self.dbuf[1..])
    }

    fn write_dram(&mut self, ddap: DDAP, slice: &[u8]) -> Result<E> {
        let mut buf = [0; SEGMENTS_SIZE + 1];
        buf[0] = ddap as u8; 
        buf[1..=slice.len()].clone_from_slice(slice);
        self.i2c.write(self.i2c_address, &buf[..=slice.len()])
    }
}
