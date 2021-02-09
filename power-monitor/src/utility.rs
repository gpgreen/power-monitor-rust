//! utilities for chart plotter hat
//! These functions are only used in debug builds

#[cfg(debug_assertions)]
use ufmt;

use chart_plotter_hat::prelude::*;
use chart_plotter_hat::hal as hal;
use hal::port::mode::Floating;

#[cfg(debug_assertions)]
pub fn send_hex_byte(serial: &mut chart_plotter_hat::Serial<Floating>, b: u8) {
    let c = match (b & 0xF0) >> 4 {
	0 => '0',
	1 => '1',
	2 => '2',
	3 => '3',
	4 => '4',
	5 => '5',
	6 => '6',
	7 => '7',
	8 => '8',
	9 => '9',
	10 => 'A',
	11 => 'B',
	12 => 'C',
	13 => 'D',
	14 => 'E',
	15 => 'F',
	_ => panic!(),
    };
    ufmt::uwrite!(serial, "{}", c).ok();
    let d = match b & 0xF {
	0 => '0',
	1 => '1',
	2 => '2',
	3 => '3',
	4 => '4',
	5 => '5',
	6 => '6',
	7 => '7',
	8 => '8',
	9 => '9',
	10 => 'A',
	11 => 'B',
	12 => 'C',
	13 => 'D',
	14 => 'E',
	15 => 'F',
	_ => panic!(),
    };
    ufmt::uwrite!(serial, "{}", d).ok();
}

#[cfg(debug_assertions)]
pub fn send_reg(serial: &mut chart_plotter_hat::Serial<Floating>, addr: u8) {
    let ptr = unsafe{ &mut *(addr as *mut u8) };
    let b = unsafe{ core::ptr::read_volatile(ptr) };
    send_hex_byte(serial, b);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}

#[cfg(debug_assertions)]
pub fn send_u16(serial: &mut chart_plotter_hat::Serial<Floating>, n: u16) {
    send_hex_byte(serial, ((n & 0xFF00) >> 8) as u8);
    send_hex_byte(serial, (n & 0xFF) as u8);
    ufmt::uwriteln!(serial, "\r").void_unwrap();
}

