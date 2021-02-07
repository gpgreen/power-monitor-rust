//!
//! Copyright 2021 Greg Green <ggreen@bit-builder.com>
//!
use chart_plotter_hat::prelude::*;
use avr_device::interrupt;
use chart_plotter_hat::hal as hal;
use hal::port::mode::{Input,PullUp,Output};

//==========================================================

const MAJOR_VERSION: u8 = 0;
const MINOR_VERSION: u8 = 4;
const MAX_ADC_PINS_USIZE: usize = 6;

//==========================================================

pub struct Registers {
    adc_values: [u16; MAX_ADC_PINS_USIZE],
    adc_channels_selected: u8,
}

pub const REGISTERS_INIT: Registers = Registers {
    adc_values: [0xFEEF, 0xFEEF, 0xFEEF, 0xFEEF, 0xFEEF, 0xFEEF],
    adc_channels_selected: 0};

static mut REGISTERSHANDLE: Registers = REGISTERS_INIT;

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
    cs: hal::port::portb::PB2<Input<PullUp>>,
    eeprom: hal::port::portb::PB7<Output>,
    led5: hal::port::portd::PD3<Output>,
    state: SpiStateMachine,
    address: u8,
    send2: u8,
}

impl SpiState {
    
    pub fn new(spidev: avr_device::atmega328p::SPI,
	       sckpin: hal::port::portb::PB5<Input<PullUp>>,
	       misopin: hal::port::portb::PB4<Output>,
	       mosipin: hal::port::portb::PB3<Input<PullUp>>,
	       cspin: hal::port::portb::PB2<Input<PullUp>>,
	       eeprompin: hal::port::portb::PB7<Output>,
	       ledpin: hal::port::portd::PD3<Output>) -> SpiState {
	SpiState {
	    spi: spidev,
	    _sck: sckpin,
	    _miso: misopin,
	    _mosi: mosipin,
	    cs: cspin,
	    eeprom: eeprompin,
	    led5: ledpin,
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
	self.spi.spcr.write(|w| w.spe().clear_bit() );
	self.led5.set_low().void_unwrap();
    }
    
    pub fn post_power_down(&mut self, cs: &interrupt::CriticalSection) {
	self.initialize(cs);
    }

    pub fn send_byte(&mut self, b: u8, _cs: &interrupt::CriticalSection) {
	unsafe { self.spi.spdr.write(|w| w.bits(b)); }
    }
    
    pub fn serial_transmission_complete(&mut self, cs: &interrupt::CriticalSection) {
	let mut change_eeprom = EEPROMState::NoChange;
	let recvd = self.spi.spdr.read().bits();
	if !self.cs.is_high().unwrap() {
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
			10 | 11 | 12 | 13 | 14 | 15 | 16 => {
			    unsafe {
				self.send_byte(REGISTERSHANDLE.get_val_lo_byte(recvd-10), cs);
				REGISTERSHANDLE.get_val_hi_byte(recvd-10)
			    }
			}
			_ => {
			    self.send_byte(0, cs);
			    0
			}
		    };
		    self.address = recvd;
		    self.led5.set_high().void_unwrap();
		    self.state = SpiStateMachine::SecondByte;
		}
		SpiStateMachine::SecondByte => {
		    self.send_byte(self.send2, cs);
		    if self.address == 0x1 {
			unsafe {
			    REGISTERSHANDLE.set_adc_channels_selected(recvd, cs);
			}
		    } else if self.address == 0x3 {
			if recvd == 0 {
			    change_eeprom = EEPROMState::TurnOff;
			} else {
			    change_eeprom = EEPROMState::TurnOn;
			}
		    }
		    self.state = SpiStateMachine::ThirdByte;
		}
		SpiStateMachine::ThirdByte => {
		    self.send_byte(0, cs);
		    self.led5.set_low().void_unwrap();
		    self.state = SpiStateMachine::Start;
		}
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
