//! routines associated with button state and button
//! press determination
//!
use avr_hal_generic::avr_device::interrupt;
use chart_plotter_hat::hal;
use chart_plotter_hat::prelude::*;
use hal::port::mode::{Floating, Input};

const BUTTON_TIMER_DURATION: i8 = 25;

//==========================================================

pub enum ButtonRelease {
    Down,
    UpShort,
    UpLong,
}

// capture button state, a mask for detecting position, and a timer
// for measuring position duration
pub struct ButtonState {
    button: hal::port::portd::PD2<Input<Floating>>,
    button_timer: i8,
    button_mask: u8,
}

impl ButtonState {
    pub fn new(buttonpin: hal::port::portd::PD2<Input<Floating>>) -> ButtonState {
        ButtonState {
            button: buttonpin,
            button_timer: -1,
            button_mask: 0xFF,
        }
    }

    pub fn timer0_overflow(&mut self, _cs: &interrupt::CriticalSection) {
        self.button_mask <<= 1;
        if self.button.is_high().unwrap() {
            self.button_mask |= 1;
        }
        if self.button_timer >= 0 && self.button_timer < i8::max_value() {
            self.button_timer += 1;
        }
    }

    // reset the state to initial conditions
    pub fn reset(&mut self, _cs: &interrupt::CriticalSection) {
        self.button_timer = -1;
    }

    // has button timer exceeded limit?
    pub fn start_timer(&mut self, _cs: &interrupt::CriticalSection) {
        self.button_timer = 0;
    }

    // has button timer exceeded limit?
    pub fn timeout(&self, _cs: &interrupt::CriticalSection) -> bool {
        self.button_timer >= BUTTON_TIMER_DURATION
    }

    // is the button down
    pub fn is_down(&self, _cs: &interrupt::CriticalSection) -> bool {
        self.button_mask == 0x00
    }

    // is the button up
    pub fn is_up(&self, _cs: &interrupt::CriticalSection) -> bool {
        self.button_mask == 0xFF
    }

    pub fn test_button_release(&mut self, cs: &interrupt::CriticalSection) -> ButtonRelease {
        if self.is_up(cs) {
            let newstate = if self.timeout(cs) {
                ButtonRelease::UpLong
            } else {
                ButtonRelease::UpShort
            };
            self.reset(cs);
            newstate
        } else {
            ButtonRelease::Down
        }
    }
}

// Required to allow static Button
unsafe impl Sync for ButtonState {}
