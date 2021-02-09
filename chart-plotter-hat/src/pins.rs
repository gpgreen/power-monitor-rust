use crate::hal::port::PortExt;

avr_hal_generic::impl_board_pins! {
    #[port_defs]
    use crate::hal::port;

    /// Generic DDR that works for all ports
    pub struct DDR {
        portb: crate::pac::PORTB,
        portc: crate::pac::PORTC,
        portd: crate::pac::PORTD,
    }

    /// Reexport of the Chart Plotter Hat's pins, with the names they have on the board
    pub struct Pins {
        /// `A0`
        ///
        /// * ADC0 (ADC input channel 0)
        /// * PCINT8 (pin change interrupt 8)
        pub a0: portc::pc0::PC0,
        /// `A1`
        ///
        /// * ADC1 (ADC input channel 1)
        /// * PCINT9 (pin change interrupt 9)
        pub a1: portc::pc1::PC1,
        /// `A2`
        ///
        /// * ADC2 (ADC input channel 2)
        /// * PCINT10 (pin change interrupt 10)
        pub a2: portc::pc2::PC2,
        /// `A3`
        ///
        /// * ADC3 (ADC input channel 3)
        /// * PCINT11 (pin change interrupt 11)
        pub a3: portc::pc3::PC3,
        /// `A4`
        ///
        /// * ADC4 (ADC input channel 4)
        /// * SDA (2-wire serial bus data input/output line)
        /// * PCINT12 (pin change interrupt 12)
        pub a4: portc::pc4::PC4,
        /// `A5`
        ///
        /// ADC5 (ADC input channel 5)
        /// SCL (2-wire serial bus clock line)
        /// PCINT13 (pin change interrupt 13)
        pub a5: portc::pc5::PC5,

        /// `RX`
        ///
        pub rx: portd::pd0::PD0,
        /// `TX`
        ///
        pub tx: portd::pd1::PD1,
        /// `BUTTON`
        ///
        pub button: portd::pd2::PD2,
        /// `EN`
        ///
        pub en: portd::pd4::PD4,
        /// `MR`
        ///
        pub mr: portb::pb1::PB1,
        /// `SHUTDOWN`
        ///
        pub shutdown: portb::pb6::PB6,
        /// `EP`
        ///
        pub ep: portb::pb7::PB7,
        /// `CS`
        ///
        pub cs: portb::pb2::PB2,
        /// `MOSI`
        ///
        pub mosi: portb::pb3::PB3,
        /// `MISO`
        ///
        pub miso: portb::pb4::PB4,
        /// `SCK`
        ///
        pub sck: portb::pb5::PB5,
        /// `LED1`
        ///
        pub led1: portb::pb0::PB0,
        /// `LED2`
        ///
        pub led2: portd::pd7::PD7,
        /// `LED3`
        ///
        pub led3: portd::pd6::PD6,
        /// `LED4`
        ///
        pub led4: portd::pd5::PD5,
        /// `LED5`
        ///
        pub led5: portd::pd3::PD3,
    }
}
