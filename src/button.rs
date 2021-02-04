pub mod button {

    use core::cell::UnsafeCell;
    use avr_device::interrupt;

    const BUTTON_TIMER_DURATION: i8 = 25;
    
    // capture button state, a mask for detecting position, and a timer
    // for measuring position duration
    pub struct ButtonState {
	button_timer: UnsafeCell<i8>,
	button_mask: UnsafeCell<u8>,
    }

    // initialize the state
    pub const BUTTON_STATE_INIT: ButtonState = ButtonState{
	button_timer: UnsafeCell::new(-1), button_mask: UnsafeCell::new(0xFF)};

    impl ButtonState {
	// by requiring a CriticalSection passed in, we are inside a CriticalSection
	// and so using unsafe is ok for this block
	pub fn timer0_overflow(&self, pin_state: bool, _cs: &interrupt::CriticalSection) {
	    unsafe {
		*self.button_mask.get() <<= 1;
		if pin_state {
		    *self.button_mask.get() |= 1;
		}
		let count = *self.button_timer.get();
		if count >= 0 && count < i8::max_value() {
		    *self.button_timer.get() += 1;
		}
	    }
	}

	// reset the state to initial conditions
	pub fn reset(&self, _cs:  &interrupt::CriticalSection) {
	    unsafe {
		*self.button_timer.get() = -1;
	    }
	}

	// has button timer exceeded limit?
	pub fn start_timer(&self, _cs:  &interrupt::CriticalSection) {
	    unsafe {
		*self.button_timer.get() = 0;
	    }
	}

	// has button timer exceeded limit?
	pub fn timeout(&self, _cs:  &interrupt::CriticalSection) -> bool {
	    unsafe {
		*self.button_timer.get() >= BUTTON_TIMER_DURATION
	    }
	}

	// is the button down
	pub fn is_down(&self, _cs: &interrupt::CriticalSection) -> bool {
	    unsafe { *self.button_mask.get() == 0x00 }
	}

	// is the button up
	pub fn is_up(&self, _cs: &interrupt::CriticalSection) -> bool {
	    unsafe { *self.button_mask.get() == 0xFF }
	}
	
    }

    // Required to allow static Button
    unsafe impl Sync for ButtonState {}

}
