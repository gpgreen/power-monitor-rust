//!
//! Copyright 2021 Greg Green <ggreen@bit-builder.com>
//!
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

// see uno-adc example to show how to get to A6 & A7 adc pins

//extern crate panic_halt;

use avr_device::interrupt;
use core::ops::DerefMut;
use core::cell::{RefCell, UnsafeCell};
use ufmt;

use chart_plotter_hat::prelude::*;
use chart_plotter_hat::hal as hal;
use hal::port::mode::{Floating,TriState};

//==========================================================
// Wait Timeout

// shared variable to hold wait timeout counter
struct WaitTimeoutCounter(UnsafeCell<i8>);

const WAIT_TIMEOUT_COUNTER_INIT: WaitTimeoutCounter = WaitTimeoutCounter(UnsafeCell::new(-1));

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

// shared variable to hold button pin, until grabbed by interrupt function
static BUTTON_GPIO: interrupt::Mutex<RefCell<Option<chart_plotter_hat::hal::port::portd::PD2<TriState>>>> =
    interrupt::Mutex::new(RefCell::new(None));

mod button;
use crate::button::{ButtonState, BUTTON_STATE_INIT};

// BUTTONSTATE is no longer 'mut' as it uses interior mutability,
// therefore it also no longer requires unsafe blocks to access
static BUTTONSTATE: ButtonState = BUTTON_STATE_INIT;

//==========================================================

mod spi;
use crate::spi::SpiState;

static SPISTATEHANDLE: interrupt::Mutex<RefCell<Option<SpiState>>> =
    interrupt::Mutex::new(RefCell::new(None));

//==========================================================

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut serial: chart_plotter_hat::Serial<hal::port::mode::Floating> =
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
    PowerDownEntry,
    PowerDownExit
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
	| PowerStateMachine::PowerDownEntry
	| PowerStateMachine::PowerDownExit = state_memo.0 {
	    (new_state, state_memo.0)
	} else {
	    (new_state, state_memo.1)
	}
}

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
	    PowerStateMachine::PowerDownEntry => { r"PowerDownEntry" }
	    PowerStateMachine::PowerDownExit => { r"PowerDownExit" }
	};
	ufmt::uwrite!(serial, "{}", s).void_unwrap();
    }

}

fn send_tuple(state: (PowerStateMachine, PowerStateMachine),
	      serial: &mut chart_plotter_hat::Serial<Floating>) {
    ufmt::uwrite!(serial, "Current:").void_unwrap();
    state.0.send(serial);
    ufmt::uwrite!(serial, " Previous:").void_unwrap();
    state.1.send(serial);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}

//==========================================================

enum ButtonRelease {
    Down,
    UpShort,
    UpLong,
}

fn test_button_release() -> ButtonRelease {
    interrupt::free(|cs| {
	if BUTTONSTATE.is_up(cs) {
	    let newstate = if BUTTONSTATE.timeout(cs) {
		ButtonRelease::UpLong
	    } else {
		ButtonRelease::UpShort
	    };
	    BUTTONSTATE.reset(cs);
	    newstate
	} else {
	    ButtonRelease::Down
	}
    })
}

//==========================================================

fn power_down_entry(cpu: &chart_plotter_hat::pac::CPU,
		    exint: &chart_plotter_hat::pac::EXINT,
		    timer0: &chart_plotter_hat::pac::TC0) {
    // set sleep mode
    cpu.smcr.write(|w| w.sm().pdown());

    // SHUTDOWN pin to input and pull-up on
    //            SHUTDOWN_DIR &= ~(_BV(SHUTDOWN));
    //            SHUTDOWN_PORT |= _BV(SHUTDOWN);

    // modules power off
    interrupt::free(|cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.pre_power_down(cs);
	}
    //            sensor_pre_power_down();
    });
    
    // turn off clocks
    cpu.prr.write(|w| w.prspi().clear_bit());
    
    // stop timer0 ovf interrupt
    timer0.timsk0.write(|w| w.toie0().clear_bit());
    
    // set INTO interrupt
    exint.eimsk.write(|w| w.int0().set_bit());

    // do the power down, if INT0 interrupt hasn't happened
    // no other interrupts are important, as the Pi will
    // be powered off in this state, so only need
    // to detect button pushes
    interrupt::disable();
    unsafe {
	if !core::ptr::read_volatile(&PENDINGINT0) {
	    // sleep enable
	    cpu.smcr.write(|w| w.se().set_bit());
	    interrupt::enable();
	    // sleep cpu
	    avr_device::asm::sleep();
	} else {
	    // INT0 pending
	    core::ptr::write_volatile(&mut PENDINGINT0, false);
	    interrupt::enable();
	}
    }

    // sleep disable
    cpu.smcr.write(|w| w.se().clear_bit());
}

//==========================================================

fn power_down_exit(cpu: &chart_plotter_hat::pac::CPU,
		   exint: &chart_plotter_hat::pac::EXINT,
		   timer0: &chart_plotter_hat::pac::TC0) {
    
    // stop INTO interrupt
    exint.eimsk.write(|w| w.int0().clear_bit());

    // set timer0 ovf interrupt
    timer0.timsk0.write(|w| w.toie0().set_bit());

    // turn on clocks
    cpu.prr.write(|w| w.prspi().set_bit());

    interrupt::free(|cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.post_power_down(cs);
	}
	//            sensor_pre_power_down();
    });
    // SHUTDOWN pin pull up off, to output
    //SHUTDOWN_PORT &= ~(_BV(SHUTDOWN));
    //SHUTDOWN_DIR |= _BV(SHUTDOWN);

    // start wakeup timer
    interrupt::free(|cs| { WAITCOUNTER.start_timer(cs); });
}

//==========================================================

#[hal::entry]
fn main() -> ! {
    let dp = chart_plotter_hat::Peripherals::take().unwrap();

    // turn off unused modules
    let cpu = dp.CPU;
    cpu.prr.write(|w| {
	w.prtim1().clear_bit();
	w.prtim2().clear_bit();
	w.prtwi().clear_bit()
    });

    let exint = dp.EXINT;
    
    let mut pins = chart_plotter_hat::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
    
    // LEDs, output
    let mut led1 = pins.led1.into_output(&mut pins.ddr);
    let mut led2 = pins.led2.into_output(&mut pins.ddr);
    let mut led3 = pins.led3.into_output(&mut pins.ddr);
    let mut led4 = pins.led4.into_output(&mut pins.ddr);
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
    let mut serial = chart_plotter_hat::Serial::<Floating>::new(
	dp.USART0,
	pins.rx,
	pins.tx.into_output(&mut pins.ddr),
	57600.into_baudrate(),
    );

    // setup spi
    let sck = pins.sck.into_pull_up_input(&mut pins.ddr);
    let miso = pins.miso.into_output(&mut pins.ddr);
    let mosi = pins.mosi.into_pull_up_input(&mut pins.ddr);
    let cs = pins.cs.into_pull_up_input(&mut pins.ddr);
    // EEPROM, output, external pullup
    let eeprom = pins.ep.into_output(&mut pins.ddr);

    let mut spi_state = SpiState::new(dp.SPI, sck, miso, mosi, cs, eeprom, led5);
    interrupt::free(|cs| {
	// transfer to static variable
	*BUTTON_GPIO.borrow(cs).borrow_mut() = Some(button_pin);

	spi_state.initialize(cs);
	// transfer to static variable
	SPISTATEHANDLE.borrow(cs).replace(Some(spi_state));
    });

    // start value of FSM
    let mut machine_state = (PowerStateMachine::Start, PowerStateMachine::Start);
    let mut prev_state = machine_state;

    // enable interrupts
    unsafe {
	interrupt::enable();
    }

    ufmt::uwriteln!(&mut serial, "PowerMonitor Start\r").void_unwrap();
    
    loop {
	// FSM
	machine_state = match machine_state.0 {
	    PowerStateMachine::Start => {
		interrupt::free(|cs| { WAITCOUNTER.start_timer(cs); });
		change_state(PowerStateMachine::WaitEntry, machine_state)
	    }
	    PowerStateMachine::WaitEntry => {
		led2.set_high().void_unwrap();
		led3.set_low().void_unwrap();
		led4.set_low().void_unwrap();
		enable_pin.set_low().void_unwrap();
		shutdown_pin.set_low().void_unwrap();
		change_state(PowerStateMachine::Wait, machine_state)
	    }
	    PowerStateMachine::Wait => {
		if interrupt::free(|cs| { BUTTONSTATE.is_down(cs) }) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else if interrupt::free(|cs| { WAITCOUNTER.timeout(cs) }) {
		    change_state(PowerStateMachine::MCUOffEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::SignaledOnEntry => {
		led2.set_low().void_unwrap();
		led3.set_high().void_unwrap();
		led4.set_low().void_unwrap();
		interrupt::free(|cs| { WAITCOUNTER.reset(cs); });
		enable_pin.set_high().void_unwrap();
		change_state(PowerStateMachine::SignaledOn, machine_state)
	    }
	    PowerStateMachine::SignaledOn => {
		if interrupt::free(|cs| { BUTTONSTATE.is_down(cs) }) {
		    change_state(PowerStateMachine::ButtonPress, machine_state)
		} else if mcu_running_pin.is_high().unwrap() {
		    change_state(PowerStateMachine::MCURunningEntry, machine_state)
		} else {
		    machine_state
		}
	    }
	    PowerStateMachine::MCURunningEntry => {
		led2.set_low().void_unwrap();
		led3.set_low().void_unwrap();
		led4.set_high().void_unwrap();
		change_state(PowerStateMachine::MCURunning, machine_state)
	    }
	    PowerStateMachine::MCURunning => {
		if interrupt::free(|cs| { BUTTONSTATE.is_down(cs) }) {
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
		led2.set_low().void_unwrap();
		led3.set_low().void_unwrap();
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
		led2.set_low().void_unwrap();
		led3.set_low().void_unwrap();
		led4.set_low().void_unwrap();
		shutdown_pin.set_low().void_unwrap();
		enable_pin.set_low().void_unwrap();
		change_state(PowerStateMachine::MCUOff, machine_state)
	    }
	    PowerStateMachine::MCUOff => {
		change_state(PowerStateMachine::PowerDownEntry, machine_state)
	    }
//	    PowerStateMachine::ADCNoiseEntry => {
//		change_state(PowerStateMachine::ADCNoiseExit, machine_state)
//	    }
//	    PowerStateMachine::ADCNoiseExit => {
//		change_state(PowerStateMachine::MCURunningEntry, machine_state)
//	    }
	    PowerStateMachine::PowerDownEntry => {
		power_down_entry(&cpu, &exint, &timer0);
		change_state(PowerStateMachine::PowerDownExit, machine_state)
	    }
	    PowerStateMachine::PowerDownExit => {
		power_down_exit(&cpu, &exint, &timer0);
		change_state(PowerStateMachine::WaitEntry, machine_state)
	    }
	    PowerStateMachine::ButtonPress => {
		interrupt::free(|cs| { BUTTONSTATE.start_timer(cs) });
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
	// check and see if machine state has changed, if so, send it via serial port
	if prev_state.0 != machine_state.0 || prev_state.1 != machine_state.1 {
	    send_tuple(machine_state, &mut serial);
	    prev_state = machine_state;
	}

//	chart_plotter_hat::delay_us(10000);
    }
}

//==========================================================

// interrupt handler for Timer0 overflow
#[interrupt(atmega328p)]
fn TIMER0_OVF() {
    // static variable to hold the button pin
    static mut BUTTONPIN: Option<chart_plotter_hat::hal::port::portd::PD2<TriState>> = None;

    // create unneeded interrupt context for static functions
    // unneeded because we are in interrupt and can't be interrupted
    // again in avr
    interrupt::free(move |cs| {

	WAITCOUNTER.increment(cs);

	unsafe {
	    // pin already transferred to this function
	    if let Some(but) = &BUTTONPIN {
		BUTTONSTATE.timer0_overflow(but.is_high().unwrap(), cs);
	    }
	    else {
		// transfer pin to this function
		BUTTONPIN.replace(BUTTON_GPIO.borrow(cs).replace(None).unwrap());
	    }
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

// interrupt handler for SPI Serial Transmission Complete
#[interrupt(atmega328p)]
fn SPI_STC() {
    // create unneeded interrupt context for static functions
    // unneeded because we are in interrupt and can't be interrupted
    // again in avr
    interrupt::free(move |cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.serial_transmission_complete(cs);
	}
    });
}
