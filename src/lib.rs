//! A port of the
//! [Adafruit Motor Shield V2 Library](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library).
//! The main and begin functions have been blended into one another a bit as for now there
//! seems to be no reason to keep them really separate. The rest of the library
//! has been translated verbatim as best as possible.
//!
//! Might separate the PWM library later, but I don't have the hardware at hand (yet).
//!
//! Original doc:
//!
//! "This is the library for the Adafruit Motor Shield V2 for Arduino.
//!  It supports DC motors & Stepper motors with microstepping as well
//!  as stacking-support. It is *not* compatible with the V1 library!
//!
//!  It will only work with <https://www.adafruit.com/products/1483>
//!
//!  Adafruit invests time and resources providing this open
//!  source code, please support Adafruit and open-source hardware
//!  by purchasing products from Adafruit!
//!
//!  Written by Limor Fried/Ladyada for Adafruit Industries.
//!  BSD license, check license.txt for more information.
//!  All text above must be included in any redistribution."

#![no_std]

use crate::adafruit_ms_pwm_servo_driver::AdafruitMSPWMServoDriver;
use embedded_hal::i2c::I2c;

pub mod adafruit_ms_pwm_servo_driver;

const DEFAULT_FREQ: u16 = 1600;
const NUM_PINS: u8 = 16;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

pub struct AdafruitMotorShieldV2<I2C> {
    _addr: u8,
    _freq: u16,
    pwm: AdafruitMSPWMServoDriver<I2C>,
}

impl<I2C, E> AdafruitMotorShieldV2<I2C>
where
    I2C: I2c<Error=E>,
{
    pub fn new(addr: u8, i2c: I2C) -> Self {
        let pwm = AdafruitMSPWMServoDriver::new(addr, i2c);
        Self {
            _addr: addr,
            _freq: DEFAULT_FREQ,
            pwm,
        }
    }

    pub fn begin(&mut self, freq: u16) -> Result<(), E> {
        self.pwm.set_pwm_freq(freq as f32)?;

        for pin in 0..NUM_PINS {
            self.pwm.set_pwm(pin, 0, 0)?;
        }

        Ok(())
    }

    pub fn set_pwm(&mut self, pin: u8, value: u16) -> Result<(), E> {
        match value > 4095 {
            true => self.pwm.set_pwm(pin, 4096, 0)?,
            false => self.pwm.set_pwm(pin, 0, value)?,
        }
        Ok(())
    }

    pub fn set_pin(&mut self, pin: u8, value: bool) -> Result<(), E> {
        match value {
            true => self.pwm.set_pwm(pin, 4096, 0)?,
            false => self.pwm.set_pwm(pin, 0, 0)?,
        }
        Ok(())
    }
}
