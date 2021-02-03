#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

extern crate panic_halt;

use arduino_uno::prelude::*;
use avr_device::interrupt;
use avr_device::interrupt::Mutex;
use core::cell::RefCell;
use avr_hal_generic::port::mode::TriState;
use atmega328p_hal::port::portd::PD2 as ButtonPin;

mod button;
use crate::button::button::{ButtonState, BUTTON_STATE_INIT};

//const CPU_FREQUENCY_HZ: u64 = 8_000_000;

// shared variable to hold button pin, until grabbed by interrupt function
static BUTTON_GPIO: Mutex<RefCell<Option<ButtonPin<TriState>>>> = Mutex::new(RefCell::new(None));

// BUTTONSTATE is no longer 'mut' as it uses interior mutability,
// therefore it also no longer requires unsafe blocks to access
static BUTTONSTATE: ButtonState = BUTTON_STATE_INIT;

#[arduino_uno::entry]
fn main() -> ! {
    let dp = arduino_uno::Peripherals::take().unwrap();

    let mut portb = dp.PORTB.split();
    let mut portd = dp.PORTD.split();
    let t0 = dp.TC0;
    
    // LED1, output
    let mut led1 = portb.pb0.into_output(&mut portb.ddr);
    led1.set_low().void_unwrap();
	
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

    avr_device::interrupt::free(move |cs| {

	// setup Timer0, CK/256, overflow interrupt enabled
	t0.tccr0b.write(|w| w.cs0().prescale_256());
	t0.timsk0.write(|w| w.toie0().set_bit());
	
	// transfer to static variable inside interrupt
	*BUTTON_GPIO.borrow(cs).borrow_mut() = Some(button_pin);
    });

    loop {
	let bdown = avr_device::interrupt::free(move |cs| { BUTTONSTATE.is_down(cs) });
	let bup = avr_device::interrupt::free(move |cs| { BUTTONSTATE.is_up(cs) });
	if bdown {
	    led1.set_high().void_unwrap();
	} else if bup {
	    led1.set_low().void_unwrap();
	}
	arduino_uno::delay_us(10000);
    }
    // unimplemented!();
}

// interrupt handler for the Timer0 overflow
#[interrupt(atmega328p)]
fn TIMER0_OVF() {
    static mut BUTTONPIN: Option<ButtonPin<TriState>> = None;
    avr_device::interrupt::free(|cs| {
	unsafe {
	    if let Some(but) = &BUTTONPIN {
		BUTTONSTATE.timer0_overflow(but.is_low().unwrap(), cs);
	    }
	    else {
		BUTTONPIN.replace(BUTTON_GPIO.borrow(cs).replace(None).unwrap());
	    }
	}
    });
}
