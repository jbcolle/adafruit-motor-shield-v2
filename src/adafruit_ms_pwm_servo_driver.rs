//! Port of the
//! [Adfruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library).
//!
//! Original doc:
//!
//! "This is a library for our Adafruit 16-channel PWM & Servo driver
//!
//!   Pick one up today in the adafruit shop!
//!   ------> <http://www.adafruit.com/products/815>
//!
//!   These displays use I2C to communicate, 2 pins are required to
//!   interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4
//!
//!   Adafruit invests time and resources providing this open source code,
//!   please support Adafruit and open-source hardware by purchasing
//!   products from Adafruit!
//!
//!   Written by Limor Fried/Ladyada for Adafruit Industries.
//!   BSD license, all text above must be included in any redistribution"

#![allow(unused)]
use arduino_hal::delay_ms;
use libm::floorf;

const PCA9685_SUBADR1: u8 = 0x2;
const PCA9685_SUBADR2: u8 = 0x3;
const PCA9685_SUBADR3: u8 = 0x4;

const PCA9685_MODE1: u8 = 0x00;
const PCA9685_PRESCALE: u8 = 0xFE;

const LED0_ON_L: u8 = 0x06;
const LED0_ON_H: u8 = 0x07;
const LED0_OFF_L: u8 = 0x08;
const LED0_OFF_H: u8 = 0x09;

const ALLLED_ON_L: u8 = 0xFA;
const ALLLED_ON_H: u8 = 0xFB;
const ALLLED_OFF_L: u8 = 0xFC;
const ALLLED_OFF_H: u8 = 0xFD;

pub struct AdafruitMSPWMServoDriver<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> AdafruitMSPWMServoDriver<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error=E>,
{
    pub fn new(addr: u8, i2c: I2C) -> Self {
        Self { i2c, addr }
    }

    pub fn set_pwm(&mut self, channel: u8, on: u16, off: u16) -> Result<(), E> {
        let base_reg = LED0_ON_L + (4 * channel);
        let on_l = (on & 0xFF) as u8;
        let on_h = (on >> 8) as u8;
        let off_l = (off & 0xFF) as u8;
        let off_h = (off >> 8) as u8;

        self.i2c
            .write(self.addr, &[base_reg, on_l, on_h, off_l, off_h])
    }

    pub fn set_pwm_freq(&mut self, freq: f32) -> Result<(), E> {
        let freq = freq * 0.9;
        let mut prescaleval = 25000000.0;
        prescaleval /= 4096.0;
        prescaleval /= freq;
        prescaleval -= 1.0;

        let prescale = floorf(prescaleval + 0.5) as u8;

        let oldmode = self.read8(PCA9685_MODE1)?;
        let new_mode = (oldmode & 0x7F) | 0x10;

        self.write8(PCA9685_MODE1, new_mode)?;
        self.write8(PCA9685_PRESCALE, prescale)?;
        self.write8(PCA9685_MODE1, oldmode)?;
        delay_ms(5);
        self.write8(PCA9685_MODE1, oldmode | 0xa1)?;

        Ok(())
    }

    fn read8(&mut self, addr: u8) -> Result<u8, E> {
        let mut buf = [0u8];
        self.i2c.write_read(self.addr, &[addr], &mut buf)?;
        Ok(buf[0])
    }

    fn write8(&mut self, addr: u8, data: u8) -> Result<(), E> {
        let buf = [addr, data];
        self.i2c.write(self.addr, &buf)
    }
}
