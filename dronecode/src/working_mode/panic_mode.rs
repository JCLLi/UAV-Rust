use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::motor::{get_motors, set_motors};
use tudelft_quadrupel::time::assembly_delay;
use crate::working_mode::WorkingModes;

//Mode switch function for panic mode
pub fn panic_mode() -> WorkingModes{
    set_motors([200,200,200,200]);
    assembly_delay(1_000);
    set_motors([0,0,0,0]);
    WorkingModes::SafeMode
}

//Different situations that can panic
pub fn panic_check() -> bool{
    //Panic when battery is low
    // if read_battery() < 1 {
    //     return false;
    // }

    //Panic when motors shut down
    for i in get_motors(){
        if i == 0{
            return false;
        }
    }

    //TODO: Add NACK panic
    true
}