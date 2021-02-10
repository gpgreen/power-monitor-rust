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
use chart_plotter_hat::prelude::*;
use chart_plotter_hat::hal as hal;
use hal::port::mode::{Analog, Floating, Input, Output};

mod button;
use crate::button::{ButtonRelease, ButtonState};

mod spi;
use crate::spi::{SpiState, REGISTERSHANDLE, MAX_ADC_PINS_I8};

#[cfg(debug_assertions)]
use ufmt;

mod utility;
#[cfg(debug_assertions)]
use utility::*;

//==========================================================

struct Hardware {
    cpu: chart_plotter_hat::pac::CPU,
    exint: chart_plotter_hat::pac::EXINT,
    timer0: chart_plotter_hat::pac::TC0,
    adc: RefCell<chart_plotter_hat::adc::Adc>,
    led1: hal::port::portb::PB0<Output>,
    mcu_running_pin: hal::port::portb::PB1<Input<Floating>>,
    enable_pin: hal::port::portd::PD4<Output>,
    shutdown_pin: hal::port::portb::PB6<Output>,
    machine_state: (PowerStateMachine, PowerStateMachine),
    prev_state: (PowerStateMachine, PowerStateMachine),
    current_channel: i8,
    a0: hal::port::portc::PC0<Analog>,
    a1: hal::port::portc::PC1<Analog>,
    a2: hal::port::portc::PC2<Analog>,
    a3: hal::port::portc::PC3<Analog>,
    a4: hal::port::portc::PC4<Analog>,
    a5: hal::port::portc::PC5<Analog>,
    #[cfg(debug_assertions)]
    serial: chart_plotter_hat::Serial<Floating>,
    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    led2: hal::port::portd::PD7<Output>,
    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    led3: hal::port::portd::PD6<Output>,
    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    led4: hal::port::portd::PD5<Output>,
}

//==========================================================

#[hal::entry]
fn main() -> ! {
    let mut hdwr = setup();
    loop {
	hdwr = process_fsm(hdwr);
	//	chart_plotter_hat::delay_us(10000);
    }
}

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

static BUTTONSTATEHANDLE: interrupt::Mutex<RefCell<Option<ButtonState>>>
    = interrupt::Mutex::new(RefCell::new(None));

//==========================================================

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

fn setup() -> Hardware {
    
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
	w.prusart0().set_bit()
    });
    // turn off analog comparator
    let ac = dp.AC;
    ac.acsr.write(|w| w.acd().set_bit() );
    
    let exint = dp.EXINT;
    
    let mut pins = chart_plotter_hat::Pins::new(dp.PORTB, dp.PORTC, dp.PORTD);
    
    // LEDs, output
    let led1 = pins.led1.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    let led2 = pins.led2.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    let led3 = pins.led3.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    let led4 = pins.led4.into_output(&mut pins.ddr);
    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    let led5 = pins.led5.into_output(&mut pins.ddr);
    
    // MCU_RUNNING, input, external pulldown
    let mcu_running_pin = pins.mr;
    
    // ENABLE, output
    let enable_pin = pins.en.into_output(&mut pins.ddr);
    
    // SHUTDOWN, output
    let shutdown_pin = pins.shutdown.into_output(&mut pins.ddr);

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
    let a0 = pins.a0.into_analog_input(&mut adc);
    let a1 = pins.a1.into_analog_input(&mut adc);
    let a2 = pins.a2.into_analog_input(&mut adc);
    let a3 = pins.a3.into_analog_input(&mut adc);
    let a4 = pins.a4.into_analog_input(&mut adc);
    let a5 = pins.a5.into_analog_input(&mut adc);

    #[cfg(feature = "proto-board")]
    #[cfg(debug_assertions)]
    let mut spi_state = SpiState::new(dp.SPI, sck, miso, mosi, cs, eeprom, led5);
    #[cfg(not(debug_assertions))]
    let mut spi_state = SpiState::new(dp.SPI, sck, miso, mosi, cs, eeprom);

    let button_state = ButtonState::new(button_pin);
    
    interrupt::free(|cs| {
	// transfer to static variable
	BUTTONSTATEHANDLE.borrow(cs).replace(Some(button_state));

	spi_state.initialize(cs);
	// transfer to static variable
	SPISTATEHANDLE.borrow(cs).replace(Some(spi_state));
    });

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
    
    Hardware {
	cpu: cpu,
	exint: exint,
	led1: led1,
	mcu_running_pin: mcu_running_pin,
	enable_pin: enable_pin,
	shutdown_pin: shutdown_pin,
	timer0: timer0,
	machine_state: (PowerStateMachine::Start, PowerStateMachine::Start),
	prev_state: (PowerStateMachine::Start, PowerStateMachine::Start),
	current_channel: -1,
	adc: RefCell::new(adc),
	a0: a0,
	a1: a1,
	a2: a2,
	a3: a3,
	a4: a4,
	a5: a5,
	#[cfg(debug_assertions)]
	serial: serial,
	#[cfg(debug_assertions)]
	#[cfg(feature = "proto-board")]
	led2: led2,
	#[cfg(debug_assertions)]
	#[cfg(feature = "proto-board")]
	led3: led3,
	#[cfg(debug_assertions)]
	#[cfg(feature = "proto-board")]
	led4: led4,
    }
}

//==========================================================

fn power_down_mode(hdwr: Hardware) -> Hardware {
    // SHUTDOWN pin to input and pull-up on
    //            SHUTDOWN_DIR &= ~(_BV(SHUTDOWN));
    //            SHUTDOWN_PORT |= _BV(SHUTDOWN);

    // release hal adc so we can turn it off
    let adcp = hdwr.adc.into_inner().release();
    
    // modules power off
    interrupt::free(|cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.pre_power_down(cs);
	}
	adcp.adcsra.reset();
    });
    
    // turn off clocks
    hdwr.cpu.prr.modify(|_, w| {
	w.prspi().set_bit();
	w.pradc().set_bit()
    });
    
    // stop timer0 ovf interrupt
    hdwr.timer0.timsk0.modify(|_, w| w.toie0().clear_bit());
    
    // set INTO interrupt
    hdwr.exint.eimsk.modify(|_, w| w.int0().set_bit());

    // do the power down, if INT0 interrupt hasn't happened
    // no other interrupts are important, as the Pi will
    // be powered off in this state, so only need
    // to detect button pushes
    hdwr.cpu.smcr.write(|w| w.sm().pdown());
    interrupt::disable();
    unsafe {
	if !core::ptr::read_volatile(&PENDINGINT0) {
	    // sleep enable
	    hdwr.cpu.smcr.modify(|_, w| w.se().set_bit());
	    interrupt::enable();
	    // sleep cpu
	    avr_device::asm::sleep();
	    // sleep disable
	    hdwr.cpu.smcr.modify(|_, w| w.se().clear_bit());
	} else {
	    // INT0 pending
	    core::ptr::write_volatile(&mut PENDINGINT0, false);
	}
	interrupt::enable();
    }

    // stop INTO interrupt
    hdwr.exint.eimsk.modify(|_, w| w.int0().clear_bit());

    // set timer0 ovf interrupt
    hdwr.timer0.timsk0.modify(|_, w| w.toie0().set_bit());

    // turn on clocks
    hdwr.cpu.prr.modify(|_, w| {
	w.prspi().clear_bit();
	w.pradc().clear_bit()
    });
    
    interrupt::free(|cs| {
	if let Some(ref mut ss) = SPISTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
	    ss.post_power_down(cs);
	}
	// start wakeup timer
	WAITCOUNTER.start_timer(cs);
    });

    // SHUTDOWN pin pull up off, to output
    //SHUTDOWN_PORT &= ~(_BV(SHUTDOWN));
    //SHUTDOWN_DIR |= _BV(SHUTDOWN);

    // start adc again
    Hardware {
	cpu: hdwr.cpu,
	exint: hdwr.exint,
	led1: hdwr.led1,
	mcu_running_pin: hdwr.mcu_running_pin,
	enable_pin: hdwr.enable_pin,
	shutdown_pin: hdwr.shutdown_pin,
	timer0: hdwr.timer0,
	machine_state: hdwr.machine_state,
	prev_state: hdwr.prev_state,
	current_channel: hdwr.current_channel,
	adc: RefCell::new(chart_plotter_hat::adc::Adc::new(adcp, Default::default())),
	a0: hdwr.a0,
	a1: hdwr.a1,
	a2: hdwr.a2,
	a3: hdwr.a3,
	a4: hdwr.a4,
	a5: hdwr.a5,
	#[cfg(debug_assertions)]
	serial: hdwr.serial,
	#[cfg(feature = "proto-board")]
	#[cfg(debug_assertions)]
	led2: hdwr.led2,
	#[cfg(feature = "proto-board")]
	#[cfg(debug_assertions)]
	led3: hdwr.led3,
	#[cfg(feature = "proto-board")]
	#[cfg(debug_assertions)]
	led4: hdwr.led4,
    }
}

//==========================================================

#[cfg(feature = "proto-board")]
#[cfg(debug_assertions)]
fn led_show_state(hdwr: &mut Hardware, current_state: PowerStateMachine) {
    match current_state {
	PowerStateMachine::WaitEntry => {
	    hdwr.led2.set_high().void_unwrap();
	    hdwr.led3.set_low().void_unwrap();
	    hdwr.led4.set_low().void_unwrap();
	},
	PowerStateMachine::SignaledOnEntry => {
	    hdwr.led2.set_low().void_unwrap();
	    hdwr.led3.set_high().void_unwrap();
	    hdwr.led4.set_low().void_unwrap();
	},
	PowerStateMachine::MCURunningEntry => {
	    hdwr.led2.set_low().void_unwrap();
	    hdwr.led3.set_low().void_unwrap();
	    hdwr.led4.set_high().void_unwrap();
	},
	PowerStateMachine::SignaledOffEntry => {
	    hdwr.led2.set_low().void_unwrap();
	    hdwr.led3.set_low().void_unwrap();
	    hdwr.led4.set_low().void_unwrap();
	},
	PowerStateMachine::MCUOffEntry => {
	    hdwr.led2.set_low().void_unwrap();
	    hdwr.led3.set_low().void_unwrap();
	    hdwr.led4.set_low().void_unwrap();
	},
	_ => (),
    }
}

//==========================================================

fn process_fsm(mut hdwr: Hardware) -> Hardware {
    // FSM
    let newstate = match hdwr.machine_state.0 {
	PowerStateMachine::Start => {
	    interrupt::free(|cs| { WAITCOUNTER.start_timer(cs); });
	    change_state(PowerStateMachine::WaitEntry, hdwr.machine_state)
	}
	PowerStateMachine::WaitEntry => {
	    #[cfg(feature = "proto-board")]
	    #[cfg(debug_assertions)]
	    led_show_state(&mut hdwr, PowerStateMachine::WaitEntry);

	    hdwr.enable_pin.set_low().void_unwrap();
	    hdwr.shutdown_pin.set_low().void_unwrap();
	    change_state(PowerStateMachine::Wait, hdwr.machine_state)
	}
	PowerStateMachine::Wait => {
	    if interrupt::free(|cs| {
		if let Some(bs) = BUTTONSTATEHANDLE.borrow(cs).borrow().deref() {
		    bs.is_down(cs)
		} else {false}
	    }) {
		change_state(PowerStateMachine::ButtonPress, hdwr.machine_state)
	    } else if interrupt::free(|cs| { WAITCOUNTER.timeout(cs) }) {
		change_state(PowerStateMachine::MCUOffEntry, hdwr.machine_state)
	    } else {
		hdwr.machine_state
	    }
	}
	PowerStateMachine::SignaledOnEntry => {
	    #[cfg(feature = "proto-board")]
	    #[cfg(debug_assertions)]
	    led_show_state(&mut hdwr, PowerStateMachine::SignaledOnEntry);

	    interrupt::free(|cs| { WAITCOUNTER.reset(cs); });
	    hdwr.enable_pin.set_high().void_unwrap();
	    change_state(PowerStateMachine::SignaledOn, hdwr.machine_state)
	}
	PowerStateMachine::SignaledOn => {
	    if interrupt::free(|cs| {
		if let Some(bs) = BUTTONSTATEHANDLE.borrow(cs).borrow().deref() {
		    bs.is_down(cs)
		} else { false }
	    }) {
		change_state(PowerStateMachine::ButtonPress, hdwr.machine_state)
	    } else if hdwr.mcu_running_pin.is_high().unwrap() {
		change_state(PowerStateMachine::MCURunningEntry, hdwr.machine_state)
	    } else {
		hdwr.machine_state
	    }
	}
	PowerStateMachine::MCURunningEntry => {
	    #[cfg(feature = "proto-board")]
	    #[cfg(debug_assertions)]
	    led_show_state(&mut hdwr, PowerStateMachine::MCURunningEntry);

	    change_state(PowerStateMachine::MCURunning, hdwr.machine_state)
	}
	PowerStateMachine::MCURunning => {
	    if interrupt::free(|cs| {
		if let Some(bs) = BUTTONSTATEHANDLE.borrow(cs).borrow().deref() {
		    bs.is_down(cs)
		} else { false }
	    }) {
		change_state(PowerStateMachine::ButtonPress, hdwr.machine_state)
	    } else if hdwr.mcu_running_pin.is_low().unwrap() {
		// this can happen if the user turns off the Pi using it's OS
		change_state(PowerStateMachine::MCUOffEntry, hdwr.machine_state)
	    } else {
		hdwr.machine_state
	    }
	}
	PowerStateMachine::SignaledOffEntry => {
	    hdwr.led1.set_high().void_unwrap();
	    #[cfg(feature = "proto-board")]
	    #[cfg(debug_assertions)]
	    led_show_state(&mut hdwr, PowerStateMachine::SignaledOffEntry);

	    hdwr.shutdown_pin.set_high().void_unwrap();
	    change_state(PowerStateMachine::SignaledOff, hdwr.machine_state)
	}
	PowerStateMachine::SignaledOff => {
	    if hdwr.mcu_running_pin.is_low().unwrap() {
		change_state(PowerStateMachine::MCUOffEntry, hdwr.machine_state)
	    } else {
		hdwr.machine_state
	    }
	}
	PowerStateMachine::MCUOffEntry => {
	    hdwr.led1.set_low().void_unwrap();
	    #[cfg(feature = "proto-board")]
	    #[cfg(debug_assertions)]
	    led_show_state(&mut hdwr, PowerStateMachine::MCUOffEntry);

	    hdwr.shutdown_pin.set_low().void_unwrap();
	    hdwr.enable_pin.set_low().void_unwrap();
	    change_state(PowerStateMachine::MCUOff, hdwr.machine_state)
	}
	PowerStateMachine::MCUOff => {
	    change_state(PowerStateMachine::PowerDown, hdwr.machine_state)
	}
	//	    PowerStateMachine::ADCNoiseEntry => {
	//		change_state(PowerStateMachine::ADCNoiseExit, hdwr.machine_state)
	//	    }
	//	    PowerStateMachine::ADCNoiseExit => {
	//		change_state(PowerStateMachine::MCURunningEntry, hdwr.machine_state)
	//	    }
	PowerStateMachine::PowerDown => {
	    #[cfg(debug_assertions)]
	    hdwr.serial.flush().ok();
	    // cpu goes to sleep, then woken up
	    hdwr = power_down_mode(hdwr);

	    #[cfg(debug_assertions)]
	    ufmt::uwrite!(hdwr.serial, "prr:").void_unwrap();
	    #[cfg(debug_assertions)]
	    send_reg(&mut hdwr.serial, 0x64);

	    change_state(PowerStateMachine::WaitEntry, hdwr.machine_state)
	}
	PowerStateMachine::ButtonPress => {
	    interrupt::free(|cs| {
		if let Some(ref mut bs) = BUTTONSTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
		    bs.start_timer(cs);
		} else { () }
	    });
	    change_state(PowerStateMachine::ButtonRelease, hdwr.machine_state)
	}
	PowerStateMachine::ButtonRelease => {
	    match interrupt::free(|cs| {
		if let Some(ref mut bs) = BUTTONSTATEHANDLE.borrow(cs).borrow_mut().deref_mut() {
		    bs.test_button_release(cs)
		} else { ButtonRelease::Down }
	    }) {
		ButtonRelease::UpLong => {
		    match hdwr.machine_state.1 {
			PowerStateMachine::Wait => {
			    change_state(PowerStateMachine::SignaledOnEntry, hdwr.machine_state)
			}
			PowerStateMachine::SignaledOn => {
			    change_state(PowerStateMachine::MCUOffEntry, hdwr.machine_state)
			}
			PowerStateMachine::MCURunning => {
			    change_state(PowerStateMachine::SignaledOffEntry, hdwr.machine_state)
			}
			_ => hdwr.machine_state
		    }
		}
		ButtonRelease::UpShort => {
		    match hdwr.machine_state.1 {
			PowerStateMachine::Wait => {
			    change_state(PowerStateMachine::WaitEntry, hdwr.machine_state)
			}
			PowerStateMachine::SignaledOn => {
			    change_state(PowerStateMachine::SignaledOnEntry, hdwr.machine_state)
			}
			PowerStateMachine::MCURunning => {
			    change_state(PowerStateMachine::MCURunningEntry, hdwr.machine_state)
			}
			_ => hdwr.machine_state
		    }
		}
		ButtonRelease::Down => {
		    hdwr.machine_state
		}
	    }
	}
    };
    // check and see if machine state has changed, if so, send it via serial port
    if hdwr.prev_state.0 != hdwr.machine_state.0 || hdwr.prev_state.1 != hdwr.machine_state.1 {
	#[cfg(debug_assertions)]
	send_tuple(hdwr.machine_state, &mut hdwr.serial);
	hdwr.prev_state = hdwr.machine_state;
    }
    // handle adc readings
    let channels = interrupt::free(|_cs|
				   unsafe {
				       REGISTERSHANDLE.get_adc_channels_selected()
				   });
    if channels != 0 {
	if hdwr.current_channel == -1 {
	    let mut i = 0;
	    while i < MAX_ADC_PINS_I8 {
		if ((1 << i) & channels) == (1 << i) {
		    hdwr.current_channel = i;
		    break;
		} else {
		    i += 1;
		}
	    }
	}
	let adc_result : core::result::Result<u16, _> = match hdwr.current_channel {
	    0 => hdwr.adc.borrow_mut().read(&mut hdwr.a0),
	    1 => hdwr.adc.borrow_mut().read(&mut hdwr.a1),
	    2 => hdwr.adc.borrow_mut().read(&mut hdwr.a2),
	    3 => hdwr.adc.borrow_mut().read(&mut hdwr.a3),
	    4 => hdwr.adc.borrow_mut().read(&mut hdwr.a4),
	    5 => hdwr.adc.borrow_mut().read(&mut hdwr.a5),
	    #[cfg(not(feature = "proto-board"))]
	    6 => hdwr.adc.borrow_mut().read(&mut chart_plotter_hat::adc::channel::ADC6),
	    #[cfg(not(feature = "proto-board"))]
	    7 => hdwr.adc.borrow_mut().read(&mut chart_plotter_hat::adc::channel::ADC7),
	    _ => Err(nb::Error::WouldBlock),
	};
	match adc_result {
	    Ok(adc_reading) => {
		
		#[cfg(debug_assertions)]
		ufmt::uwrite!(hdwr.serial, "adc_val:").void_unwrap();
		#[cfg(debug_assertions)]
		send_u16(&mut hdwr.serial, adc_reading);
		
		interrupt::free(|cs| unsafe {
		    REGISTERSHANDLE.set_value(
			hdwr.current_channel.try_into().unwrap(), adc_reading, cs);
		});
		let mut i = hdwr.current_channel + 1;
		while i != hdwr.current_channel {
		    while i < MAX_ADC_PINS_I8 {
			if ((1 << i) & channels) == (1 << i) {
			    hdwr.current_channel = i;
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
    Hardware {
	cpu: hdwr.cpu,
	exint: hdwr.exint,
	led1: hdwr.led1,
	mcu_running_pin: hdwr.mcu_running_pin,
	enable_pin: hdwr.enable_pin,
	shutdown_pin: hdwr.shutdown_pin,
	timer0: hdwr.timer0,
	machine_state: newstate,
	prev_state: hdwr.prev_state,
	current_channel: hdwr.current_channel,
	adc: hdwr.adc,
	a0: hdwr.a0,
	a1: hdwr.a1,
	a2: hdwr.a2,
	a3: hdwr.a3,
	a4: hdwr.a4,
	a5: hdwr.a5,
	#[cfg(debug_assertions)]
	serial: hdwr.serial,
	#[cfg(feature = "proto-board")]
	#[cfg(debug_assertions)]
	led2: hdwr.led2,
	#[cfg(feature = "proto-board")]
	#[cfg(debug_assertions)]
	led3: hdwr.led3,
	#[cfg(feature = "proto-board")]
	#[cfg(debug_assertions)]
	led4: hdwr.led4,
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
