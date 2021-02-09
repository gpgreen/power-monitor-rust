//! firmware for chart plotter hat
//! monitors a push button to turn on/off a Raspberry Pi
//! Also implements adc conversions based on an spi interface

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

extern crate panic_halt;

use avr_device::interrupt;
use core::ops::{Deref,DerefMut};
use core::cell::{RefCell, UnsafeCell};
use core::convert::TryInto;
use nb;

#[cfg(debug_assertions)]
use ufmt;

use chart_plotter_hat::prelude::*;
use chart_plotter_hat::hal as hal;
use hal::port::mode::Floating;

//==========================================================

mod utility;
use utility::*;

//==========================================================
// Wait Timeout

// shared variable to hold wait timeout counter
struct WaitTimeoutCounter(UnsafeCell<i8>);

const WAIT_TIMEOUT_COUNTER_INIT: WaitTimeoutCounter
    = WaitTimeoutCounter(UnsafeCell::new(-1));

impl WaitTimeoutCounter {
    pub fn reset(&self, _cs: &interrupt::CriticalSection) {
	unsafe { *self.0.get() = -1 };
    }

    pub fn increment(&self, _cs: &interrupt::CriticalSection) {
	unsafe {
	    let cnt = *self.0.get();
	    if cnt >= 0 && cnt < i8::max_value() {
		*self.0.get() += 1;
	    }
	}
    }

    pub fn start_timer(&self, _cs: &interrupt::CriticalSection) {
	unsafe { *self.0.get() = 0 };
    }
    
    pub fn timeout(&self, _cs: &interrupt::CriticalSection) -> bool {
	unsafe { *self.0.get() >= 81 }
    }
}

unsafe impl Sync for WaitTimeoutCounter {}

static WAITCOUNTER: WaitTimeoutCounter = WAIT_TIMEOUT_COUNTER_INIT;

//==========================================================
// INT0 Flag

static mut PENDINGINT0: bool = false;

//==========================================================

mod button;
use crate::button::{ButtonRelease, ButtonState};

// BUTTONSTATEHANDLE is no longer 'mut' as it uses interior mutability,
// therefore it also no longer requires unsafe blocks to access
static BUTTONSTATEHANDLE: interrupt::Mutex<RefCell<Option<ButtonState>>>
    = interrupt::Mutex::new(RefCell::new(None));

//==========================================================

mod spi;
use crate::spi::{SpiState, REGISTERSHANDLE, MAX_ADC_PINS_I8};

static SPISTATEHANDLE: interrupt::Mutex<RefCell<Option<SpiState>>> =
    interrupt::Mutex::new(RefCell::new(None));

//==========================================================

#[derive(Copy, Clone, PartialEq)]
enum PowerStateMachine {
    Start,
    WaitEntry,
    Wait,
    ButtonPress,
    ButtonRelease,
    SignaledOnEntry,
    SignaledOn,
    MCURunningEntry,
    MCURunning,
//    ADCNoiseEntry,
//    ADCNoiseExit,
    SignaledOffEntry,
    SignaledOff,
    MCUOffEntry,
    MCUOff,
    PowerDown,
}

fn change_state(new_state: PowerStateMachine,
		state_memo: (PowerStateMachine, PowerStateMachine))
		-> (PowerStateMachine, PowerStateMachine) {
    if let PowerStateMachine::Start
	| PowerStateMachine::Wait
	| PowerStateMachine::SignaledOn
	| PowerStateMachine::MCURunning
//	| PowerStateMachine::ADCNoiseExit
	| PowerStateMachine::SignaledOff
	| PowerStateMachine::MCUOff
	| PowerStateMachine::PowerDown = state_memo.0 {
	    (new_state, state_memo.0)
	} else {
	    (new_state, state_memo.1)
	}
}

#[cfg(debug_assertions)]
impl PowerStateMachine {
    fn send(&self, serial: &mut chart_plotter_hat::Serial<Floating>) {
	let s = match self {
	    PowerStateMachine::Start => { r"Start" }
	    PowerStateMachine::WaitEntry => { r"WaitEntry" }
	    PowerStateMachine::Wait => { r"Wait" }
	    PowerStateMachine::ButtonPress => { r"ButtonPress" }
	    PowerStateMachine::ButtonRelease => { r"ButtonRelease" }
	    PowerStateMachine::SignaledOnEntry => { r"SignaledOnEntry" }
	    PowerStateMachine::SignaledOn => { r"SignaledOn" }
	    PowerStateMachine::MCURunningEntry => { r"MCURunningEntry" }
	    PowerStateMachine::MCURunning => { r"MCURunning" }
//	    PowerStateMachine::ADCNoiseEntry => { r"ADCNoiseEntry" }
//	    PowerStateMachine::ADCNoiseExit => { r"ADCNoiseExit" }
	    PowerStateMachine::SignaledOffEntry => { r"SignaledOffEntry" }
	    PowerStateMachine::SignaledOff => { r"SignaledOff" }
	    PowerStateMachine::MCUOffEntry => { r"MCUOffEntry" }
	    PowerStateMachine::MCUOff => { r"MCUOff" }
	    PowerStateMachine::PowerDown => { r"PowerDown" }
	};
	ufmt::uwrite!(serial, "{}", s).void_unwrap();
    }

}

#[cfg(debug_assertions)]
fn send_tuple(state: (PowerStateMachine, PowerStateMachine),
	      serial: &mut chart_plotter_hat::Serial<Floating>) {
    ufmt::uwrite!(serial, "Current:").void_unwrap();
    state.0.send(serial);
    ufmt::uwrite!(serial, " Previous:").void_unwrap();
    state.1.send(serial);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}

//==========================================================

fn power_down_mode(cpu: &chart_plotter_hat::pac::CPU,
		   exint: &chart_plotter_hat::pac::EXINT,
		   timer0: &chart_plotter_hat::pac::TC0,
		   adc: chart_plotter_hat::adc::Adc) -> chart_plotter_hat::adc::Adc {
    // SHUTDOWN pin to input and pull-up on
    //            SHUTDOWN_DIR &= ~(_BV(SHUTDOWN));
    //            SHUTDOWN_PORT |= _BV(SHUTDOWN);

    // release hal adc so we can turn it off
    let adcp = adc.release();
    
    // modules power off
    interrupt::free(|cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.pre_power_down(cs);
	}
	adcp.adcsra.reset();
    });
    
    // turn off clocks
    cpu.prr.modify(|_, w| {
	w.prspi().set_bit();
	w.pradc().set_bit()
    });
    
    // stop timer0 ovf interrupt
    timer0.timsk0.modify(|_, w| w.toie0().clear_bit());
    
    // set INTO interrupt
    exint.eimsk.modify(|_, w| w.int0().set_bit());

    // do the power down, if INT0 interrupt hasn't happened
    // no other interrupts are important, as the Pi will
    // be powered off in this state, so only need
    // to detect button pushes
    cpu.smcr.write(|w| w.sm().pdown());
    interrupt::disable();
    unsafe {
	if !core::ptr::read_volatile(&PENDINGINT0) {
	    // sleep enable
	    cpu.smcr.modify(|_, w| w.se().set_bit());
	    interrupt::enable();
	    // sleep cpu
	    avr_device::asm::sleep();
	    // sleep disable
	    cpu.smcr.modify(|_, w| w.se().clear_bit());
	} else {
	    // INT0 pending
	    core::ptr::write_volatile(&mut PENDINGINT0, false);
	}
	interrupt::enable();
    }

    // stop INTO interrupt
    exint.eimsk.modify(|_, w| w.int0().clear_bit());

    // set timer0 ovf interrupt
    timer0.timsk0.modify(|_, w| w.toie0().set_bit());

    // turn on clocks
    cpu.prr.modify(|_, w| {
	w.prspi().clear_bit();
	w.pradc().clear_bit()
    });
    
    interrupt::free(|cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.post_power_down(cs);
	}
    });

    // start wakeup timer
    interrupt::free(|cs| { WAITCOUNTER.start_timer(cs); });

    // SHUTDOWN pin pull up off, to output
    //SHUTDOWN_PORT &= ~(_BV(SHUTDOWN));
    //SHUTDOWN_DIR |= _BV(SHUTDOWN);

    // start adc again
    chart_plotter_hat::adc::Adc::new(adcp, Default::default())
}

//==========================================================

#[hal::entry]
fn main() -> ! {
    let dp = chart_plotter_hat::Peripherals::take().unwrap();

    // turn off unused modules
    let cpu = dp.CPU;
    cpu.prr.write(|w| {
	w.prtim1().set_bit();
	w.prtim2().set_bit();
	w.prtwi().set_bit()
    });
    #[cfg(not(debug_assertions))]
    cpu.prr.write(|w| {
	w.prusart().set_bit();
    });
    // turn off analog comparator
    let ac = dp.AC;
    ac.acsr.write(|w| w.acd().set_bit() );
    
    let exint = dp.EXINT;
    
    let mut pins = chart_plotter_hat::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
    
    // LEDs, output
    let mut led1 = pins.led1.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    let mut led2 = pins.led2.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    let mut led3 = pins.led3.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    let mut led4 = pins.led4.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    let led5 = pins.led5.into_output(&mut pins.ddr);
    
    // MCU_RUNNING, input, external pulldown
    let mcu_running_pin = pins.mr;
	
    // ENABLE, output
    let mut enable_pin = pins.en.into_output(&mut pins.ddr);
    
    // SHUTDOWN, output
    let mut shutdown_pin = pins.shutdown.into_output(&mut pins.ddr);

    // BUTTON, tri-state input and output, external pullup
    let button_pin = pins.button.into_tri_state(&mut pins.ddr);
    
    // setup Timer0, CK/256, overflow interrupt enabled
    let timer0 = dp.TC0;
    timer0.tccr0b.write(|w| w.cs0().prescale_256());
    timer0.timsk0.write(|w| w.toie0().set_bit());

    // setup serial
    #[cfg(debug_assertions)]
    let mut serial = chart_plotter_hat::Serial::<Floating>::new(
	dp.USART0,
	pins.rx,
	pins.tx.into_output(&mut pins.ddr),
	57600.into_baudrate(),
    );

    // SPI pins
    let sck = pins.sck.into_pull_up_input(&mut pins.ddr);
    let miso = pins.miso.into_output(&mut pins.ddr);
    let mosi = pins.mosi.into_pull_up_input(&mut pins.ddr);
    let cs = pins.cs.into_pull_up_input(&mut pins.ddr);
    // EEPROM, output, external pullup
    let eeprom = pins.ep.into_output(&mut pins.ddr);

    // setup adc, default is 128 clock division, and AVcc voltage reference
    let mut adc = chart_plotter_hat::adc::Adc::new(dp.ADC, Default::default());
    let mut a0 = pins.a0.into_analog_input(&mut adc);
    let mut a1 = pins.a1.into_analog_input(&mut adc);
    let mut a2 = pins.a2.into_analog_input(&mut adc);
    let mut a3 = pins.a3.into_analog_input(&mut adc);
    let mut a4 = pins.a4.into_analog_input(&mut adc);
    let mut a5 = pins.a5.into_analog_input(&mut adc);

    #[cfg(feature = "proto-board")]
    let mut spi_state = SpiState::new(dp.SPI, sck, miso, mosi, cs, eeprom, led5);
    #[cfg(not(feature = "proto-board"))]
    let mut spi_state = SpiState::new(dp.SPI, sck, miso, mosi, cs, eeprom);

    let button_state = ButtonState::new(button_pin);
    
    interrupt::free(|cs| {
	// transfer to static variable
	BUTTONSTATEHANDLE.borrow(cs).replace(Some(button_state));

	spi_state.initialize(cs);
	// transfer to static variable
	SPISTATEHANDLE.borrow(cs).replace(Some(spi_state));
    });

    // start value of FSM
    let mut machine_state = (PowerStateMachine::Start, PowerStateMachine::Start);
    let mut prev_state = machine_state;

    let mut current_channel : i8 = -1;
    
    // enable interrupts
    unsafe {
	interrupt::enable();
    }

    #[cfg(debug_assertions)]
    ufmt::uwriteln!(&mut serial, "\r\nPowerMonitor Start\r").void_unwrap();
    
    #[cfg(debug_assertions)]
    ufmt::uwrite!(serial, "spcr:").void_unwrap();
    #[cfg(debug_assertions)]
    send_reg(&mut serial, 0x4c);
    #[cfg(debug_assertions)]
    ufmt::uwrite!(serial, "prr:").void_unwrap();
    #[cfg(debug_assertions)]
    send_reg(&mut serial, 0x64);

    loop {
	// FSM
	machine_state = match machine_state.0 {
	    PowerStateMachine::Start => {
		interrupt::free(|cs| { WAITCOUNTER.start_timer(cs); });
		change_state(PowerStateMachine::WaitEntry, machine_state)
	    }
	    PowerStateMachine::WaitEntry => {
		#[cfg(feature = "proto-board")]
		led2.set_high().void_unwrap();
		#[cfg(feature = "proto-board")]
		led3.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led4.set_low().void_unwrap();
		enable_pin.set_low().void_unwrap();
		shutdown_pin.set_low().void_unwrap();
		change_state(PowerStateMachine::Wait, machine_state)
	    }
	    PowerStateMachine::Wait => {
		if interrupt::free(|cs| {
		    if let Some(bs) = BUTTONSTATEHANDLE.borrow(cs).borrow().deref() {
			bs.is_down(cs)
		    } else {false}
		}) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else if interrupt::free(|cs| { WAITCOUNTER.timeout(cs) }) {
		    change_state(PowerStateMachine::MCUOffEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::SignaledOnEntry => {
		#[cfg(feature = "proto-board")]
		led2.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led3.set_high().void_unwrap();
		#[cfg(feature = "proto-board")]
		led4.set_low().void_unwrap();
		interrupt::free(|cs| { WAITCOUNTER.reset(cs); });
		enable_pin.set_high().void_unwrap();
		change_state(PowerStateMachine::SignaledOn, machine_state)
	    }
	    PowerStateMachine::SignaledOn => {
		if interrupt::free(|cs| {
		    if let Some(bs) = BUTTONSTATEHANDLE.borrow(cs).borrow().deref() {
			bs.is_down(cs)
		    } else { false }
		}) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else if mcu_running_pin.is_high().unwrap() {
		    change_state(PowerStateMachine::MCURunningEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::MCURunningEntry => {
		#[cfg(feature = "proto-board")]
		led2.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led3.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led4.set_high().void_unwrap();
		change_state(PowerStateMachine::MCURunning, machine_state)
	    }
	    PowerStateMachine::MCURunning => {
		if interrupt::free(|cs| {
		    if let Some(bs) = BUTTONSTATEHANDLE.borrow(cs).borrow().deref() {
			bs.is_down(cs)
		    } else { false }
		}) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else if mcu_running_pin.is_low().unwrap() {
		    // this can happen if the user turns off the Pi using it's OS
		     change_state(PowerStateMachine::MCUOffEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::SignaledOffEntry => {
		led1.set_high().void_unwrap();
		#[cfg(feature = "proto-board")]
		led2.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led3.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led4.set_low().void_unwrap();
		shutdown_pin.set_high().void_unwrap();
		change_state(PowerStateMachine::SignaledOff, machine_state)
	    }
	    PowerStateMachine::SignaledOff => {
		if mcu_running_pin.is_low().unwrap() {
		    change_state(PowerStateMachine::MCUOffEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::MCUOffEntry => {
		led1.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led2.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led3.set_low().void_unwrap();
		#[cfg(feature = "proto-board")]
		led4.set_low().void_unwrap();
		shutdown_pin.set_low().void_unwrap();
		enable_pin.set_low().void_unwrap();
		change_state(PowerStateMachine::MCUOff, machine_state)
	    }
	    PowerStateMachine::MCUOff => {
		change_state(PowerStateMachine::PowerDown, machine_state)
	    }
//	    PowerStateMachine::ADCNoiseEntry => {
//		change_state(PowerStateMachine::ADCNoiseExit, machine_state)
//	    }
//	    PowerStateMachine::ADCNoiseExit => {
//		change_state(PowerStateMachine::MCURunningEntry, machine_state)
//	    }
	    PowerStateMachine::PowerDown => {
		#[cfg(debug_assertions)]
		serial.flush().ok();
		// cpu goes to sleep, then woken up
		adc = power_down_mode(&cpu, &exint, &timer0, adc);

		#[cfg(debug_assertions)]
		ufmt::uwrite!(serial, "prr:").void_unwrap();
		#[cfg(debug_assertions)]
		send_reg(&mut serial, 0x64);

		change_state(PowerStateMachine::WaitEntry, machine_state)
	    }
	    PowerStateMachine::ButtonPress => {
		interrupt::free(|cs| {
		    if let Some(ref mut bs) = BUTTONSTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
			bs.start_timer(cs);
		    } else { () }
		});
		change_state(PowerStateMachine::ButtonRelease, machine_state)
	    }
	    PowerStateMachine::ButtonRelease => {
		match interrupt::free(|cs| {
		    if let Some(ref mut bs) = BUTTONSTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
			bs.test_button_release(cs)
		    } else { ButtonRelease::Down }
		}) {
		    ButtonRelease::UpLong => {
			match machine_state.1 {
			    PowerStateMachine::Wait => {
				change_state(PowerStateMachine::SignaledOnEntry, machine_state)
			    }
			    PowerStateMachine::SignaledOn => {
				change_state(PowerStateMachine::MCUOffEntry, machine_state)
			    }
			    PowerStateMachine::MCURunning => {
				change_state(PowerStateMachine::SignaledOffEntry, machine_state)
			    }
			    _ => machine_state
			}
		    }
		    ButtonRelease::UpShort => {
			match machine_state.1 {
			    PowerStateMachine::Wait => {
				change_state(PowerStateMachine::WaitEntry, machine_state)
			    }
			    PowerStateMachine::SignaledOn => {
				change_state(PowerStateMachine::SignaledOnEntry, machine_state)
			    }
			    PowerStateMachine::MCURunning => {
				change_state(PowerStateMachine::MCURunningEntry, machine_state)
			    }
			    _ => machine_state
			}
		    }
		    ButtonRelease::Down => {
			machine_state
		    }
		}
	    }
	};
	// check and see if machine state has changed, if so, send it via serial port
	if prev_state.0 != machine_state.0 || prev_state.1 != machine_state.1 {
	    #[cfg(debug_assertions)]
	    send_tuple(machine_state, &mut serial);
	    prev_state = machine_state;
	}
	// handle adc readings
	let channels = interrupt::free(|_cs|
				       unsafe {
					   REGISTERSHANDLE.get_adc_channels_selected()
				       });
	if channels != 0 {
	    if current_channel == -1 {
		let mut i = 0;
		while i < MAX_ADC_PINS_I8 {
		    if ((1 << i) & channels) == (1 << i) {
			current_channel = i;
			break;
		    } else {
			i += 1;
		    }
		}
	    }
	    let adc_result : core::result::Result<u16, _> = match current_channel {
		0 => adc.read(&mut a0),
		1 => adc.read(&mut a1),
		2 => adc.read(&mut a2),
		3 => adc.read(&mut a3),
		4 => adc.read(&mut a4),
		5 => adc.read(&mut a5),
		#[cfg(not(feature = "proto-board"))]
		6 => adc.read(&mut chart_plotter_hat::adc::channel::ADC6),
		#[cfg(not(feature = "proto-board"))]
		7 => adc.read(&mut chart_plotter_hat::adc::channel::ADC7),
		_ => Err(nb::Error::WouldBlock),
	    };
	    match adc_result {
		Ok(adc_reading) => {

		    #[cfg(debug_assertions)]
		    ufmt::uwrite!(serial, "adc_val:").void_unwrap();
		    #[cfg(debug_assertions)]
		    send_u16(&mut serial, adc_reading);

		    interrupt::free(|cs| unsafe {
			REGISTERSHANDLE.set_value(
			    current_channel.try_into().unwrap(), adc_reading, cs);
		    });
		    let mut i = current_channel + 1;
		    while i != current_channel {
			while i < MAX_ADC_PINS_I8 {
			    if ((1 << i) & channels) == (1 << i) {
				current_channel = i;
				break;
			    } else {
				i += 1;
			    }
			}
			if i == MAX_ADC_PINS_I8 {
			    i = 0;
			}
		    };
		},
		Err(_) => (),
	    }
	}
//	chart_plotter_hat::delay_us(10000);
    }
}

//==========================================================

// interrupt handler for Timer0 overflow
#[interrupt(atmega328p)]
fn TIMER0_OVF() {
    // create unneeded interrupt context for static functions
    // unneeded because we are in interrupt and can't be interrupted
    // again in avr
    interrupt::free(move |cs| {

	WAITCOUNTER.increment(cs);

	if let Some(ref mut bs) = BUTTONSTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    bs.timer0_overflow(cs);
	}
    });
}

//==========================================================

// interrupt handler for INT0
#[interrupt(atmega328p)]
fn INT0() {
    unsafe {
	core::ptr::write_volatile(&mut PENDINGINT0, true);
    }
}

//==========================================================

// interrupt handler for SPI Transmission Complete
#[interrupt(atmega328p)]
fn SPI_STC() {
    // create unneeded interrupt context for static functions
    // unneeded because we are in interrupt and can't be interrupted
    // again in avr
    interrupt::free(move |cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.spi_transmission_complete(cs);
	}
    });
}
