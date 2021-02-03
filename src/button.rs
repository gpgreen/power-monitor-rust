pub mod button {

    use core::cell::UnsafeCell;
    use avr_device::interrupt;

    pub struct ButtonState {
	button_timer: UnsafeCell<i8>,
	button_mask: UnsafeCell<u8>,
    }

    pub const BUTTON_STATE_INIT: ButtonState = ButtonState{
	button_timer: UnsafeCell::new(-1), button_mask: UnsafeCell::new(0)};

    impl ButtonState {
	// by requiring a CriticalSection passed in, we are inside a CriticalSection
	// and so using unsafe is ok for this block
	pub fn timer0_overflow(&self, pin_state: bool, _cs: &interrupt::CriticalSection) {
	    unsafe {
		*self.button_mask.get() <<= 1;
		if pin_state {
		    *self.button_mask.get() |= 1;
		}
		if *self.button_timer.get() >= 0 {
		    *self.button_timer.get() += 1;
		}
	    }
	}

	pub fn reset(&self, _cs:  &interrupt::CriticalSection) {
	    unsafe {
		*self.button_mask.get() = 0xFF;
		*self.button_timer.get() = -1;
	    }
	}
	
	pub fn is_down(&self, _cs: &interrupt::CriticalSection) -> bool {
	    unsafe { *self.button_mask.get() == 0x00 }
	}

	pub fn is_up(&self, _cs: &interrupt::CriticalSection) -> bool {
	    unsafe { *self.button_mask.get() == 0xFF }
	}
	
    }

    // Required to allow static Button
    unsafe impl Sync for ButtonState {}

}
