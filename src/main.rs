#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

// see uno-adc example to show how to get to A6 & A7 adc pins

//extern crate panic_halt;

use arduino_uno::prelude::*;
use avr_device::interrupt;
use core::cell::RefCell;
use avr_hal_generic::port::mode::{Floating,Output,TriState};
use ufmt;

use atmega328p_hal as hal;
use hal::port::portb::PB0 as LED1Pin;
use hal::port::portd::PD7 as LED2Pin;
use hal::port::portd::PD6 as LED3Pin;
use hal::port::portd::PD5 as LED4Pin;
use hal::port::portd::PD3 as LED5Pin;
use hal::port::portd::PD2 as ButtonPin;

pub use crate::hal::pac;
pub use crate::hal::usart;

mod button;
use crate::button::button::{ButtonState, BUTTON_STATE_INIT};

//const CPU_FREQUENCY_HZ: u64 = 8_000_000;

// shared variable to hold button pin, until grabbed by interrupt function
static BUTTON_GPIO: interrupt::Mutex<RefCell<Option<ButtonPin<TriState>>>> =
    interrupt::Mutex::new(RefCell::new(None));

// BUTTONSTATE is no longer 'mut' as it uses interior mutability,
// therefore it also no longer requires unsafe blocks to access
static BUTTONSTATE: ButtonState = BUTTON_STATE_INIT;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut serial: arduino_uno::Serial<arduino_uno::hal::port::mode::Floating> =
        unsafe { core::mem::MaybeUninit::uninit().assume_init() };

    ufmt::uwriteln!(&mut serial, "Firmware panic!\r").void_unwrap();

    if let Some(loc) = info.location() {
        ufmt::uwriteln!(
            &mut serial,
            "  At {}:{}:{}\r",
            loc.file(),
            loc.line(),
            loc.column(),
        )
        .void_unwrap();
    }

    loop {}
}

struct LEDS {
    led1: LED1Pin<Output>,
    led2: LED2Pin<Output>,
    led3: LED3Pin<Output>,
    led4: LED4Pin<Output>,
    led5: LED5Pin<Output>,
}

impl LEDS {
    fn on_all(&mut self) {
	self.led1.set_high().void_unwrap();
	self.led2.set_high().void_unwrap();
	self.led3.set_high().void_unwrap();
	self.led4.set_high().void_unwrap();
	self.led5.set_high().void_unwrap();
    }
    
    fn off_all(&mut self) {
	self.led1.set_low().void_unwrap();
	self.led2.set_low().void_unwrap();
	self.led3.set_low().void_unwrap();
	self.led4.set_low().void_unwrap();
	self.led5.set_low().void_unwrap();
    }
    
    fn on(&mut self, n: u8) {
	match n {
	    1 => self.led1.set_high().void_unwrap(),
	    2 => self.led2.set_high().void_unwrap(),
	    3 => self.led3.set_high().void_unwrap(),
	    4 => self.led4.set_high().void_unwrap(),
	    5 => self.led5.set_high().void_unwrap(),
	    _ => (),
	}
    }

    fn off(&mut self, n: u8) {
	match n {
	    1 => self.led1.set_low().void_unwrap(),
	    2 => self.led2.set_low().void_unwrap(),
	    3 => self.led3.set_low().void_unwrap(),
	    4 => self.led4.set_low().void_unwrap(),
	    5 => self.led5.set_low().void_unwrap(),
	    _ => (),
	}
    }
}

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
    ADCNoiseEntry,
    ADCNoiseExit,
    SignaledOffEntry,
    SignaledOff,
    MCUOffEntry,
    MCUOff,
    PowerDownEntry,
    PowerDownExit
}

impl PowerStateMachine {
    fn send(&self, serial: &mut hal::usart::Usart0::<hal::clock::MHz8, Floating>) {
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
	    PowerStateMachine::ADCNoiseEntry => { r"ADCNoiseEntry" }
	    PowerStateMachine::ADCNoiseExit => { r"ADCNoiseExit" }
	    PowerStateMachine::SignaledOffEntry => { r"SignaledOffEntry" }
	    PowerStateMachine::SignaledOff => { r"SignaledOff" }
	    PowerStateMachine::MCUOffEntry => { r"MCUOffEntry" }
	    PowerStateMachine::MCUOff => { r"MCUOff" }
	    PowerStateMachine::PowerDownEntry => { r"PowerDownEntry" }
	    PowerStateMachine::PowerDownExit => { r"PowerDownExit" }
	};
	ufmt::uwrite!(serial, "{}", s).void_unwrap();
    }

}

fn send_tuple(state: (PowerStateMachine, PowerStateMachine),
	      serial: &mut hal::usart::Usart0::<hal::clock::MHz8, Floating>) {
    ufmt::uwrite!(serial, "Current:").void_unwrap();
    state.0.send(serial);
    ufmt::uwrite!(serial, " Previous:").void_unwrap();
    state.1.send(serial);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}

enum ButtonRelease {
    Down,
    UpShort,
    UpLong,
}

fn test_button_release() -> ButtonRelease {
    interrupt::free(move |cs| {
	let state = BUTTONSTATE.is_up(cs);
	let to = BUTTONSTATE.timeout(cs);
	if state {
	    BUTTONSTATE.reset(cs);
	    if to {
		ButtonRelease::UpLong
	    } else {
		ButtonRelease::UpShort
	    }
	} else {
	    ButtonRelease::Down
	}
    })
}

fn change_state(new_state: PowerStateMachine, state: (PowerStateMachine, PowerStateMachine)) -> (PowerStateMachine, PowerStateMachine) {
    let old_state = state.1;
    if let PowerStateMachine::Start
	| PowerStateMachine::Wait
	| PowerStateMachine::SignaledOn
	| PowerStateMachine::MCURunning
	| PowerStateMachine::ADCNoiseExit
	| PowerStateMachine::SignaledOff
	| PowerStateMachine::MCUOffEntry
	| PowerStateMachine::MCUOff
	| PowerStateMachine::PowerDownExit = state.0 {
	    (new_state, state.0)
	} else {
	    (new_state, old_state)
	}
}

#[arduino_uno::entry]
fn main() -> ! {
    let dp = arduino_uno::Peripherals::take().unwrap();

    let mut portb = dp.PORTB.split();
    let mut portd = dp.PORTD.split();
    let timer0 = dp.TC0;
    
    // LEDs, output
    let mut leds = LEDS {
	led1: portb.pb0.into_output(&mut portb.ddr),
	led2: portd.pd7.into_output(&mut portd.ddr),
	led3: portd.pd6.into_output(&mut portd.ddr),
	led4: portd.pd5.into_output(&mut portd.ddr),
	led5: portd.pd3.into_output(&mut portd.ddr),
    };
    
    // MCU_RUNNING, input, no pullup
    let mcu_running_pin = portb.pb1;
	
    // ENABLE, output
    let mut enable_pin = portd.pd4.into_output(&mut portd.ddr);
    enable_pin.set_low().void_unwrap();
    
    // SHUTDOWN, output
    let mut shutdown_pin = portb.pb6.into_output(&mut portb.ddr);
    shutdown_pin.set_low().void_unwrap();

    // BUTTON, tri-state input and output
    let button_pin = portd.pd2.into_tri_state(&mut portd.ddr);

    // setup Timer0, CK/256, overflow interrupt enabled
    timer0.tccr0b.write(|w| w.cs0().prescale_256());
    timer0.timsk0.write(|w| w.toie0().set_bit());

    // setup serial
    let mut serial = hal::usart::Usart0::<hal::clock::MHz8, Floating>::new(
	dp.USART0,
	portd.pd0,
	portd.pd1.into_output(&mut portd.ddr),
	57600.into_baudrate(),
    );
    
    interrupt::free(move |cs| {
	// transfer to static variable inside interrupt
	*BUTTON_GPIO.borrow(cs).borrow_mut() = Some(button_pin);
    });

    // starting value of FSM
    let mut machine_state = (PowerStateMachine::Start, PowerStateMachine::Start);
    let mut prev_state = machine_state;

    ufmt::writeln!(&mut serial, "PowerMonitor Start\r").void_unwrap();
    
    // enable interrupts
    unsafe {
	interrupt::enable();
    }

    loop {
	machine_state = match machine_state.0 {
	    PowerStateMachine::Start => {
		change_state(PowerStateMachine::WaitEntry, machine_state)
	    }
	    PowerStateMachine::WaitEntry => {
		enable_pin.set_low().void_unwrap();
		shutdown_pin.set_low().void_unwrap();
		change_state(PowerStateMachine::Wait, machine_state)
	    }
	    PowerStateMachine::Wait => {
		if interrupt::free(move |cs| { BUTTONSTATE.is_down(cs) }) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::SignaledOnEntry => {
		enable_pin.set_high().void_unwrap();
		change_state(PowerStateMachine::SignaledOn, machine_state)
	    }
	    PowerStateMachine::SignaledOn => {
		if interrupt::free(move |cs| { BUTTONSTATE.is_down(cs) }) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else if mcu_running_pin.is_high().unwrap() {
		    change_state(PowerStateMachine::MCURunningEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::MCURunningEntry => {
		change_state(PowerStateMachine::MCURunning, machine_state)
	    }
	    PowerStateMachine::MCURunning => {
		if interrupt::free(move |cs| { BUTTONSTATE.is_down(cs) }) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else if mcu_running_pin.is_low().unwrap() {
		    // this can happen if the user turns off the Pi using it's OS
		     change_state(PowerStateMachine::MCUOffEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::SignaledOffEntry => {
		leds.on(1);
		shutdown_pin.set_high().void_unwrap();
		change_state(PowerStateMachine::SignaledOffEntry, machine_state)
	    }
	    PowerStateMachine::SignaledOff => {
		if mcu_running_pin.is_low().unwrap() {
		    leds.off(1);
		    shutdown_pin.set_low().void_unwrap();
		    change_state(PowerStateMachine::MCUOffEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::MCUOffEntry => {
		enable_pin.set_low().void_unwrap();
		change_state(PowerStateMachine::MCUOff, machine_state)
	    }
	    PowerStateMachine::MCUOff => {
		change_state(PowerStateMachine::PowerDownEntry, machine_state)
	    }
	    PowerStateMachine::ADCNoiseEntry => {
		change_state(PowerStateMachine::ADCNoiseExit, machine_state)
	    }
	    PowerStateMachine::ADCNoiseExit => {
		change_state(PowerStateMachine::MCURunningEntry, machine_state)
	    }
	    PowerStateMachine::PowerDownEntry => {
		change_state(PowerStateMachine::PowerDownExit, machine_state)
	    }
	    PowerStateMachine::PowerDownExit => {
		change_state(PowerStateMachine::WaitEntry, machine_state)
	    }
	    PowerStateMachine::ButtonPress => {
		interrupt::free(move |cs| { BUTTONSTATE.start_timer(cs) });
		change_state(PowerStateMachine::ButtonRelease, machine_state)
	    }
	    PowerStateMachine::ButtonRelease => {
		match test_button_release() {
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
	if prev_state.0 != machine_state.0 || prev_state.1 != machine_state.1 {
	    send_tuple(machine_state, &mut serial);
	    prev_state = machine_state;
	}

	//	if interrupt::free(move |cs| { BUTTONSTATE.is_down(cs) }) {
//	    interrupt::free(move |cs| { BUTTONSTATE.start_timer(cs) });
//	    leds.on_all();
//	} else if interrupt::free(move |cs| { BUTTONSTATE.is_up(cs) })
//	    && interrupt::free(move |cs| { BUTTONSTATE.timeout(cs) }) {
//		interrupt::free(move |cs| { BUTTONSTATE.reset(cs) });
//		leds.off_all();
//	    }
//	arduino_uno::delay_us(10000);
//	ufmt::uwriteln!(&mut serial, "Hello!\r").void_unwrap();
    }
}

// interrupt handler for the Timer0 overflow
#[interrupt(atmega328p)]
fn TIMER0_OVF() {
    static mut BUTTONPIN: Option<ButtonPin<TriState>> = None;
    interrupt::free(|cs| {
	unsafe {
	    if let Some(but) = &BUTTONPIN {
		BUTTONSTATE.timer0_overflow(but.is_high().unwrap(), cs);
	    }
	    else {
		BUTTONPIN.replace(BUTTON_GPIO.borrow(cs).replace(None).unwrap());
	    }
	}
    });
}
