# Power Monitor firmwre for Chart Plotter Hat

## Build instructions

Install Rust nightly.
```
rustup toolchain install nightly-2021-01-07
```

Then run:

```
cargo +nightly-2021-01-07 build --target avr-atmega328p.json -Z build-std=core --release
```

The final ELF executable file will then be available at `target/avr-atmega328p/release/template-bin.elf`.

## Chart Plotter Hat Hardware Changes

Rev B
LED1 attached to PB0
LED removed from SHUTDOWN
SHUTDOWN added pulldown
MCU_RUNNING moved from PD3 to PB1
MCU_RUNNING added pulldown
LC circuit added to AVCC
Optional 12V ADC measurement to ADC7

Rev A
Original

## PINOUTS

                ATmega328P-28P3            
            +--------------------+         
            |                    |         
      RESET-|1  PC6        PC5 29|-ADC5    
           -|2  PD0        PC4 27|-ADC4    
           -|3  PD1        PC3 26|-ADC3    
     BUTTON-|4  PD2        PC2 25|-ADC2    
       LED5-|5  PD3        PC1 24|-ADC1    
     ENABLE-|6  PD4        PC0 23|-ADC0    
        VCC-|7                 22|-GND     
        GND-|8                 21|-AREF    
   SHUTDOWN-|9  PB6            20|-AVCC    
     EEPROM-|10 PB7        PB5 19|-SCK     
       LED4-|11 PD5        PB4 18|-MISO    
       LED3-|12 PD6        PB3 17|-MOSI    
       LED2-|13 PD7        PB2 16|-CS
       LED1-|14 PB0        PB1 15|-MCU_RUNNING
            |                    |         
            +--------------------+         
         
                   ATmega328P-32A
            +--------------------------+        
            |                          |        
        3.3-|4  VCC              PB0 12|-LED1   
        3.3-|18 AVCC             PB1 13|-MCU_RUNNING
        CAP-|20 AREF             PB2 14|-CS
        GND-|3  GND              PB3 15|-MOSI   
            |                    PB4 16|-MISO   
         A6-|19 ADC6             PB5 17|-SCK    
         A7-|22 ADC7             PB6  7|-SHUTDOWN
            |                    PB7  8|-EEPROM 
           -|30 PD0                    |        
           -|31 PD1              PC0 23|-A0     
     BUTTON-|32 PD2              PC1 24|-A1     
           -|1  PD3              PC2 25|-A2     
         EN-|2  PD4              PC3 26|-A3     
           -|9  PD5              PC4 27|-A4     
           -|10 PD6              PC5 28|-A5     
           -|11 PD7              PC6 29|-RESET  
            |                          |        
            +--------------------------+        


