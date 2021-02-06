
use chart_plotter_hat::prelude::*;
use avr_device::interrupt;
use chart_plotter_hat::hal as hal;
use hal::port::mode::{Input,PullUp,Output};

const MAJOR_VERSION: u8 = 0;
const MINOR_VERSION: u8 = 4;
const MAX_ADC_PINS: u8 = 6;

enum SpiStateMachine {
    Start,
    SecondByte,
    ThirdByte,
}

pub struct SpiState {
    spi: avr_device::atmega328p::SPI,
    _sck: hal::port::portb::PB5<Input<PullUp>>,
    _miso: hal::port::portb::PB4<Output>,
    _mosi: hal::port::portb::PB3<Input<PullUp>>,
    cs: hal::port::portb::PB2<Input<PullUp>>,
    eeprom: hal::port::portb::PB7<Output>,
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
	       eeprompin: hal::port::portb::PB7<Output>) -> SpiState {
	SpiState {
	    spi: spidev,
	    _sck: sckpin,
	    _miso: misopin,
	    _mosi: mosipin,
	    cs: cspin,
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
    }

    pub fn pre_power_down(&mut self, _cs: &interrupt::CriticalSection) {
	self.spi.spcr.write(|w| w.spe().clear_bit() );
    }
    
    pub fn post_power_down(&mut self, cs: &interrupt::CriticalSection) {
	self.initialize(cs);
    }

    pub fn send_byte(&mut self, b: u8, _cs: &interrupt::CriticalSection) {
	unsafe { self.spi.spdr.write(|w| w.bits(b)); }
    }
    
    pub fn serial_transmission_complete(&mut self, cs: &interrupt::CriticalSection) {
	let mut toggle_eeprom = false;
	let recvd = self.spi.spdr.read().bits();
	if !self.cs.is_high().unwrap() {
	    match self.state {
		SpiStateMachine::Start => {
		    self.address = recvd;
		    if self.address >= 0x10 && self.address < MAX_ADC_PINS + 0x10 {
			self.send_byte(0, cs); // adc_values low byte
			self.send2 = 0;			   // adc_values hi byte
		    } else if self.address == 0x02 {
			self.send_byte(0, cs); // adc_channels
			self.send2 = 0;
		    } else if self.address == 0x03 {
			self.send_byte(0, cs);
			self.send2 = 0;
			toggle_eeprom = true;
		    } else if self.address == 0x04 {
			self.send_byte(MAJOR_VERSION, cs);
			self.send2 = MINOR_VERSION;
		    }
		    self.state = SpiStateMachine::SecondByte;
		}
		SpiStateMachine::SecondByte => {
		    self.send_byte(self.send2, cs);
		    if self.address == 0x1 {
			// adc_channels = recvd;
		    }
		    self.state = SpiStateMachine::ThirdByte;
		}
		SpiStateMachine::ThirdByte => {
		    self.send_byte(0, cs);
		    self.state = SpiStateMachine::Start;
		}
	    }
	}
	if toggle_eeprom {
	    // toggle eeprom line
	}
    }
}

// required to allow static state
unsafe impl Sync for SpiState {}
