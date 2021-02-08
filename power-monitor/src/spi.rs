//! This device will act as an SPI slave. It reads data from the
//! internal ADC These can then be retrieved by a SPI master
//! device. The device interface is register based. The SPI protocol
//! involves the master sending a byte that is the register address,
//! and then 2 more bytes. The slave returns a value in the second byte
//! corresponding to that value
//! IMPORTANT -- pulse BUTTON low for 5us to wake up AVR from ADC noise reduction
//! mode or SPI won't be turned on.
//! Delay 50us after pulse of BUTTON
//! Delay 40us after sending the address byte before sending the next byte
//! Delay 20us between the second and last byte
//! Delay 10us after the last byte before trying another transaction.
//! These delays should give the AVR enough time to respond to the interrupts
//! and prepare for the next byte or transaction.
//!
//! REGISTERS
//! 0x01 = turn on adc channels
//!   second byte contains flags of which adc channels are requested, ie if bit 0 is set
//!   then adc channel 0 is requested and so on up to 6 channels.
//!   Third byte is zero
//! 0x02 = adc channels reading
//!   second byte contains which channels are activated
//!   third byte is zero
//! 0x03 = turn on/off EEPROM pin
//!   second byte contains 1 to turn on, 0 to turn off
//!   third byte is zero
//! 0x04 = firmware version
//!  first byte is major, second byte is minor
//! 0x10-0x16 = retrieve adc value on the channel [address - 16], ie address 0x10 is adc channel 0
//!
//! SPI protocol is implemented using a state machine, transitions happen during
//! spi transfer complete interrupt, which happens when a byte is received over SPI
//!
//!   Start = waiting for address byte
//!   SecondByte = waiting for second byte
//!   ThirdByte = waiting for third byte
//!

use chart_plotter_hat::prelude::*;
use avr_device::interrupt;
use chart_plotter_hat::hal as hal;
use hal::port::mode::{Input,PullUp,Output};

//==========================================================

const MAJOR_VERSION: u8 = 0;
const MINOR_VERSION: u8 = 4;
#[cfg(feature = "proto-board")]
pub const MAX_ADC_PINS_USIZE: usize = 6;
#[cfg(feature = "proto-board")]
pub const MAX_ADC_PINS_I8: i8 = 6;
#[cfg(not(feature = "proto-board"))]
pub const MAX_ADC_PINS_USIZE: usize = 8;
#[cfg(not(feature = "proto-board"))]
pub const MAX_ADC_PINS_I8: i8 = 8;

//==========================================================

pub struct Registers {
    adc_values: [u16; MAX_ADC_PINS_USIZE],
    adc_channels_selected: u8,
}

pub const REGISTERS_INIT: Registers = Registers {
    #[cfg(feature = "proto-board")]
    adc_values: [0xFEEF, 0xFEEF, 0xFEEF, 0xFEEF, 0xFEEF, 0xFEEF],
    #[cfg(not(feature = "proto-board"))]
    adc_values: [0, 0, 0, 0, 0, 0, 0, 0],
    adc_channels_selected: 0};

pub static mut REGISTERSHANDLE: Registers = REGISTERS_INIT;

impl Registers {

    pub fn set_value(&mut self, address: u8, value: u16, _cs: &interrupt::CriticalSection) {
	self.adc_values[address as usize] = value;
    }

    pub fn get_val_hi_byte(&self, address: u8) -> u8 {
	((self.adc_values[address as usize] & 0xFF00) >> 8) as u8
    }

    pub fn get_val_lo_byte(&self, address: u8) -> u8 {
	(self.adc_values[address as usize] & 0xFF) as u8
    }

    pub fn set_adc_channels_selected(&mut self, new_selected: u8, _cs: &interrupt::CriticalSection) {
	self.adc_channels_selected = new_selected;
    }

    pub fn get_adc_channels_selected(&self) -> u8 {
	self.adc_channels_selected
    }
}

//==========================================================

enum SpiStateMachine {
    Start,
    SecondByte,
    ThirdByte,
}

//==========================================================

enum EEPROMState {
    TurnOff,
    TurnOn,
    NoChange,
}

//==========================================================

pub struct SpiState {
    spi: avr_device::atmega328p::SPI,
    _sck: hal::port::portb::PB5<Input<PullUp>>,
    _miso: hal::port::portb::PB4<Output>,
    _mosi: hal::port::portb::PB3<Input<PullUp>>,
    _cs: hal::port::portb::PB2<Input<PullUp>>,
    eeprom: hal::port::portb::PB7<Output>,
    state: SpiStateMachine,
    address: u8,
    send2: u8,
    #[cfg(feature = "proto-board")]
    led5: hal::port::portd::PD3<Output>,
}

impl SpiState {
    
    #[cfg(feature = "proto-board")]
    pub fn new(spidev: avr_device::atmega328p::SPI,
	       sckpin: hal::port::portb::PB5<Input<PullUp>>,
	       misopin: hal::port::portb::PB4<Output>,
	       mosipin: hal::port::portb::PB3<Input<PullUp>>,
	       cspin: hal::port::portb::PB2<Input<PullUp>>,
	       eeprompin: hal::port::portb::PB7<Output>,
	       led5pin: hal::port::portd::PD3<Output>,
    ) -> SpiState {
	SpiState {
	    spi: spidev,
	    _sck: sckpin,
	    _miso: misopin,
	    _mosi: mosipin,
	    _cs: cspin,
	    eeprom: eeprompin,
	    state: SpiStateMachine::Start,
	    address: 0,
	    send2: 0,
	    led5: led5pin,
	}
    }

    #[cfg(not(feature = "proto-board"))]
    pub fn new(spidev: avr_device::atmega328p::SPI,
	       sckpin: hal::port::portb::PB5<Input<PullUp>>,
	       misopin: hal::port::portb::PB4<Output>,
	       mosipin: hal::port::portb::PB3<Input<PullUp>>,
	       cspin: hal::port::portb::PB2<Input<PullUp>>,
	       eeprompin: hal::port::portb::PB7<Output>,
    ) -> SpiState {
	SpiState {
	    spi: spidev,
	    _sck: sckpin,
	    _miso: misopin,
	    _mosi: mosipin,
	    _cs: cspin,
	    eeprom: eeprompin,
	    state: SpiStateMachine::Start,
	    address: 0,
	    send2: 0,
	}
    }

    pub fn initialize(&mut self, _cs: &interrupt::CriticalSection) {
	self.spi.spcr.write(|w| {
	    w.spe().set_bit();
	    w.spie().set_bit()
	});
	self.eeprom.set_high().void_unwrap();
    }

    pub fn pre_power_down(&mut self, _cs: &interrupt::CriticalSection) {
	self.spi.spcr.write(|w| {
	    w.spe().clear_bit();
	    w.spie().clear_bit()
	});
	#[cfg(feature = "proto-board")]
	self.led5.set_low().void_unwrap();
    }
    
    pub fn post_power_down(&mut self, cs: &interrupt::CriticalSection) {
	self.initialize(cs);
    }

    pub fn send_byte(&mut self, b: u8, _cs: &interrupt::CriticalSection) {
	unsafe { self.spi.spdr.write(|w| w.bits(b)); }
    }
    
    pub fn serial_transmission_complete(&mut self, cs: &interrupt::CriticalSection) {
	let recvd = self.spi.spdr.read().bits();
	let mut change_eeprom = EEPROMState::NoChange;
	match self.state {
	    SpiStateMachine::Start => {
		self.send2 = match recvd {
		    1 => {
			self.send_byte(0, cs);
			0
		    }
		    2 => {
			unsafe {
			    self.send_byte(REGISTERSHANDLE.get_adc_channels_selected(), cs);
			}
			0
		    }
		    3 => {
			self.send_byte(0, cs);
			0
		    }
		    4 => {
			self.send_byte(MAJOR_VERSION, cs);
			MINOR_VERSION
		    }
		    16 | 17 | 18 | 19 | 20 | 21 | 22 => {
			unsafe {
			    self.send_byte(REGISTERSHANDLE.get_val_lo_byte(recvd - 16), cs);
			    REGISTERSHANDLE.get_val_hi_byte(recvd - 16)
			}
		    }
		    #[cfg(not(feature="proto-board"))]
		    23 | 24 => {
			unsafe {
			    self.send_byte(REGISTERSHANDLE.get_val_lo_byte(recvd - 16), cs);
			    REGISTERSHANDLE.get_val_hi_byte(recvd - 16)
			}
		    }
		    _ => {
			self.send_byte(0, cs);
			0
		    }
		};
		self.address = recvd;
		self.state = SpiStateMachine::SecondByte;
		#[cfg(feature = "proto-board")]
		self.led5.set_high().void_unwrap();
	    }
	    SpiStateMachine::SecondByte => {
		self.send_byte(self.send2, cs);
		match self.address {
		    1 => {
			unsafe {
			    REGISTERSHANDLE.set_adc_channels_selected(recvd, cs);
			}
		    },
		    3 => {
			if recvd == 0 {
			    change_eeprom = EEPROMState::TurnOff;
			} else {
			    change_eeprom = EEPROMState::TurnOn;
			}
		    },
		    _ => (),
		}
		self.state = SpiStateMachine::ThirdByte;
	    }
	    SpiStateMachine::ThirdByte => {
		self.send_byte(0, cs);
		self.state = SpiStateMachine::Start;
		#[cfg(feature = "proto-board")]
		self.led5.set_low().void_unwrap();
	    }
	}
	match change_eeprom {
	    EEPROMState::TurnOn => self.eeprom.set_high().void_unwrap(),
	    EEPROMState::TurnOff => self.eeprom.set_low().void_unwrap(),
	    EEPROMState::NoChange => (),
	}
    }
}

// required to allow static state
unsafe impl Sync for SpiState {}

//==========================================================
