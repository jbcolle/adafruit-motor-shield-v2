#![no_std]
#![no_main]

use adafruit_motor_shield_v2::AdafruitMotorShieldV2;
use arduino_hal::prelude::*;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        1600,
    );

    let mut adafruit_motorshield = AdafruitMotorShieldV2::new(0x60, i2c);

    ufmt::uwriteln!(&mut serial, "Set mode test:\r").unwrap_infallible();
    let res = adafruit_motorshield.begin(1600);
    match res {
        Ok(_) => ufmt::uwriteln!(&mut serial, "Ok").unwrap_infallible(),
        Err(_) => ufmt::uwriteln!(&mut serial, "Err").unwrap_infallible(),
    }

    loop {
        for i in (0..4094).step_by(15) {
            let res = adafruit_motorshield.set_pwm(15, i);
            if res.is_err() {
                break;
            }
            arduino_hal::delay_ms(100);
        }
    }
}
