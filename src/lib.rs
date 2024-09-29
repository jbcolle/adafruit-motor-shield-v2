#![no_std]

use crate::adafruit_ms_pwm_servo_driver::AdafruitMSPWMServoDriver;
use embedded_hal::i2c::I2c;

pub mod adafruit_ms_pwm_servo_driver;

const DEFAULT_ADDR: u8 = 0x60;
const DEFAULT_FREQ: u16 = 1600;
const NUM_PINS: u8 = 16;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

pub struct AdafruitMotorShieldV2<I2C> {
    addr: u8,
    freq: u16,
    pwm: AdafruitMSPWMServoDriver<I2C>,
}

impl<I2C, E> AdafruitMotorShieldV2<I2C>
where
    I2C: I2c<Error=E>,
{
    pub fn new(addr: u8, i2c: I2C) -> Self {
        let pwm = AdafruitMSPWMServoDriver::new(addr, i2c);
        Self {
            addr,
            freq: DEFAULT_FREQ,
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
