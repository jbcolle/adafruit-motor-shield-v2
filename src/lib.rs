#![no_std]
use embedded_hal::i2c::I2c;

pub mod adafruit_ms_pwm_servo_driver;

const I2C_DEFAULT_ADDR: u8 = 0x60;

struct AdafruitMotorShieldV2 {
    i2c_addr: u8,
    freq: u16,
}


