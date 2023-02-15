use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::send_bytes;

pub fn control_loop() -> ! {
    set_tick_frequency(1);
    let mut last = Instant::now();

    loop {
        let _ = Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        send_bytes(format!("DTT: {:?}ms\n", dt.as_millis()).as_bytes());

        let motors = get_motors();
        send_bytes(
            format!(
                "MTR: {} {} {} {}\n",
                motors[0], motors[1], motors[2], motors[3]
            )
            .as_bytes(),
        );

        let quaternion = read_dmp_bytes().unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        send_bytes(format!("YPR {} {} {}\n", ypr.yaw, ypr.pitch, ypr.roll).as_bytes());

        let (accel, _) = read_raw().unwrap();
        send_bytes(format!("ACC {} {} {}\n", accel.x, accel.y, accel.z).as_bytes());

        let bat = read_battery();
        send_bytes(format!("BAT {bat}\n").as_bytes());

        let pres = read_pressure();
        send_bytes(format!("BAR {pres} \n").as_bytes());

        send_bytes("\n".as_bytes());

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
}
