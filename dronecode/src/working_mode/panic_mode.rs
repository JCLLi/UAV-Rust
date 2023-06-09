
use tudelft_quadrupel::motor::{get_motors, set_motors};
use tudelft_quadrupel::time::assembly_delay;
use tudelft_quadrupel::battery::read_battery;
use protocol::WorkingModes;

//Mode switch function for panic mode
pub fn panic_mode() -> WorkingModes{
    // for i in 10..1 {
    //     set_motors([i*20,i*20,i*20,i*20]);
    //     assembly_delay(1_000_000);
    // }
    let motors = get_motors();
    set_motors([motors[0]*3/4,motors[1]*3/4,motors[2]*3/4,motors[3]*3/4]);
    assembly_delay(5_000_000);  
    set_motors([motors[0]/2,motors[1]/2,motors[2]/2,motors[3]/2]);
    assembly_delay(5_000_000);  
    set_motors([motors[0]/4,motors[1]/4,motors[2]/4,motors[3]/4]);
    assembly_delay(5_000_000);  
    set_motors([0,0,0,0]);

    WorkingModes::SafeMode
}

//Different situations that can panic
pub fn panic_check() -> bool{ 
    // Panic when battery is low
    let volt = read_battery();

    if volt < 900 && volt > 50 {
        return false;
    }

    true
}